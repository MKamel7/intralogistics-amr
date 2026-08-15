"""Assign transport orders to vehicles over VDA 5050, and notice when one dies.

WHAT A DISPATCHER IS FOR

A fleet is not several robots, it is the thing that decides which robot does
what and what happens when one stops answering. Everything interesting is in
the second half: a vehicle that takes an order and then falls over must not
take the warehouse's throughput with it.

WHY THE ASSIGNMENT RULE IS DELIBERATELY DULL

Nearest free vehicle wins. Not a bidding protocol, not a cost model with tuned
weights, because with two vehicles in one building any of those would be
unfalsifiable decoration: the measurement that would tell them apart needs a
fleet large enough for the assignment to matter, and this one is not.

What IS measured is throughput against a single vehicle, which is the only
claim a two robot fleet can actually support.

STATE COMES FROM THE VEHICLES, NEVER FROM MEMORY

The dispatcher does not track where it thinks a robot is. It reads the state
topic each vehicle publishes and acts on that. A dispatcher holding its own
belief about a fleet is a dispatcher that will eventually route two vehicles
into the same aisle because it has not noticed one of them stopped.

OFFLINE IS A FIRST CLASS OUTCOME

The connection topic carries the broker's last will. A vehicle that goes
CONNECTIONBROKEN has its order reassigned rather than left assigned to a robot
that cannot execute it, and that reassignment is what the fleet claim rests on.
"""

import json
import math
import time

try:
    import paho.mqtt.client as mqtt
except ImportError:                                   # pragma: no cover
    mqtt = None

from amr_fleet import vda5050 as v


class Vehicle:
    """What the dispatcher knows about one robot, all of it from its topics."""

    def __init__(self, manufacturer, serial):
        self.manufacturer = manufacturer
        self.serial = serial
        self.online = False
        self.x = self.y = 0.0
        self.driving = False
        self.order_id = ''
        self.nodes_left = 0
        self.battery = 100.0
        self.last_seen = 0.0
        self.errors = []

    @property
    def key(self):
        return f'{self.manufacturer}/{self.serial}'

    @property
    def free(self):
        """Idle, online, and not carrying an order it has not finished."""
        return self.online and self.nodes_left == 0 and not self.driving

    def distance_to(self, x, y):
        return math.hypot(self.x - x, self.y - y)


class Dispatcher:
    """Assigns orders and reassigns them when a vehicle stops answering.

    No ROS. The dispatcher is a fleet component and talks to vehicles the way a
    real master control does, over the same MQTT interface an integrator would
    use, so it exercises the vehicle side rather than bypassing it.
    """

    def __init__(self, broker='localhost', port=1883, stale_after=10.0):
        self.vehicles = {}
        self.pending = []          # orders waiting for a vehicle
        self.assigned = {}         # vehicle key -> order
        self.completed = []
        self.stale_after = stale_after
        self.client = None
        self.broker, self.port = broker, port
        self._order_seq = 0

    # ---- fleet state -----------------------------------------------------

    def track(self, manufacturer, serial):
        veh = Vehicle(manufacturer, serial)
        self.vehicles[veh.key] = veh
        return veh

    def on_connection(self, key, payload):
        veh = self.vehicles.get(key)
        if veh is None:
            return
        state = payload.get('connectionState')
        was = veh.online
        veh.online = state == 'ONLINE'
        veh.last_seen = time.time()
        if was and not veh.online:
            # A VEHICLE THAT DROPS TAKES ITS ORDER BACK TO THE QUEUE. Leaving
            # it assigned is how a fleet quietly loses a job: the order is
            # neither running nor waiting, and nothing reports it.
            self.reclaim(key, reason=f'connection {state}')

    def on_state(self, key, payload):
        veh = self.vehicles.get(key)
        if veh is None:
            return
        veh.last_seen = time.time()
        veh.online = True
        pos = payload.get('agvPosition') or {}
        veh.x, veh.y = float(pos.get('x', 0.0)), float(pos.get('y', 0.0))
        veh.driving = bool(payload.get('driving'))
        veh.order_id = payload.get('orderId', '')
        veh.nodes_left = len(payload.get('nodeStates', []))
        veh.battery = float((payload.get('batteryState') or {}).get('batteryCharge', 100.0))
        veh.errors = payload.get('errors', [])
        if veh.nodes_left == 0 and key in self.assigned:
            order = self.assigned.pop(key)
            if order['orderId'] == veh.order_id:
                self.completed.append(order)

    def reclaim(self, key, reason):
        order = self.assigned.pop(key, None)
        if order is not None:
            self.pending.insert(0, order)
            return order
        return None

    def sweep(self, now=None):
        """Vehicles that have stopped publishing are offline, will or no will.

        The last will covers a clean broker disconnect. It does NOT cover a
        vehicle whose process is alive and wedged, which this project has
        produced more than once: the node held its connection and published
        nothing for thirty seconds. Silence is the more reliable signal.
        """
        now = now if now is not None else time.time()
        dropped = []
        for key, veh in self.vehicles.items():
            if veh.online and veh.last_seen and now - veh.last_seen > self.stale_after:
                veh.online = False
                dropped.append(key)
                self.reclaim(key, reason='silent')
        return dropped

    # ---- assignment ------------------------------------------------------

    def submit(self, name, x, y):
        self._order_seq += 1
        order = {
            'orderId': f'{name}-{self._order_seq}',
            'orderUpdateId': 0,
            'nodes': [{
                'nodeId': name, 'sequenceId': 0, 'released': True,
                'nodePosition': {'x': float(x), 'y': float(y)},
                'actions': [],
            }],
            'edges': [],
        }
        self.pending.append(order)
        return order

    def assign(self):
        """Nearest free vehicle takes the front of the queue.

        Returns the assignments made, so a caller can measure them rather than
        infer them from the broker.
        """
        made = []
        for order in list(self.pending):
            pos = order['nodes'][0]['nodePosition']
            free = [v for v in self.vehicles.values() if v.free]
            if not free:
                break
            best = min(free, key=lambda v: v.distance_to(pos['x'], pos['y']))
            self.pending.remove(order)
            self.assigned[best.key] = order
            best.nodes_left = len(order['nodes'])   # optimistic until state says otherwise
            made.append((best.key, order))
            if self.client is not None:
                self.client.publish(
                    v.topic(best.manufacturer, best.serial, 'order'),
                    json.dumps(order), qos=1)
        return made

    # ---- transport -------------------------------------------------------

    def connect(self):                                # pragma: no cover
        if mqtt is None:
            raise RuntimeError('paho-mqtt is not installed')
        self.client = mqtt.Client(client_id='fleet-dispatcher')
        self.client.on_message = self._on_message
        self.client.connect(self.broker, self.port, keepalive=15)
        for veh in self.vehicles.values():
            for name in ('connection', 'state'):
                self.client.subscribe(v.topic(veh.manufacturer, veh.serial, name), qos=1)
        self.client.loop_start()

    def _on_message(self, _c, _u, msg):               # pragma: no cover
        try:
            payload = json.loads(msg.payload)
        except ValueError:
            return
        parts = msg.topic.split('/')
        key = f'{parts[2]}/{parts[3]}'
        if msg.topic.endswith('/connection'):
            self.on_connection(key, payload)
        elif msg.topic.endswith('/state'):
            self.on_state(key, payload)
