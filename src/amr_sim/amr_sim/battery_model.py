#!/usr/bin/env python3
"""Simulated battery and state of charge.

Publishes sensor_msgs/BatteryState so the fleet layer can make charge-aware
decisions and so energy per task becomes a measurable quantity rather than an
assumption.

CALIBRATION, NOT VALIDATION
---------------------------
The power model is FITTED to three figures the platform sheet publishes:

    standby (robot on, idle)            22 h
    active operation, no payload      17.4 h
    active operation, maximum payload   13 h

from a 1.63 kWh pack. Three numbers, three parameters, so the model reproduces
all three by construction. That is calibration and it is NOT independent
evidence that the model is right; it cannot be cited as validation of the energy
behaviour. What it does buy is that state of charge falls at a rate anchored to
a real pack, so any energy-per-task figure derived later is at least the right
order of magnitude and is traceable to a published source.

An honest gap in the source: the sheet does not say what duty cycle "active
operation time" assumes. Continuous driving at top speed for 13 h would be
93.6 km on 1.63 kWh, which is implausibly efficient for a machine of this class,
so it is clearly not that. This model therefore defines a REFERENCE SPEED at
which the published active figures are taken to apply, defaulting to half of top
speed, and says so. Change `reference_speed` and the fitted coefficients change
with it; the published runtimes are still reproduced at that speed.

Discharge is linear in energy. There is no cell chemistry here: no voltage sag,
no temperature dependence, no capacity fade over the rated 3000 cycles. Those
would be inventions, since the sheet gives none of them.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from nav_msgs.msg import Odometry


class BatteryModel(Node):
    def __init__(self):
        super().__init__('battery_model')

        # Platform figures. Defaults are the MiR250-class spec values; the
        # launch passes them from the platform spec so there is one source.
        self.declare_parameter('capacity_wh', 1630.0)
        self.declare_parameter('nominal_voltage', 47.7)
        self.declare_parameter('max_speed', 2.0)
        self.declare_parameter('max_payload_kg', 250.0)

        # Published runtimes the power model is fitted to.
        self.declare_parameter('runtime_standby_h', 22.0)
        self.declare_parameter('runtime_unloaded_h', 17.4)
        self.declare_parameter('runtime_loaded_h', 13.0)

        # The undefined bit of the source, made explicit and adjustable.
        self.declare_parameter('reference_speed', 1.0)

        self.declare_parameter('initial_soc', 1.0)
        self.declare_parameter('payload_kg', 0.0)
        self.declare_parameter('publish_rate', 1.0)
        # Real time per simulated second, so a demo can drain a pack without
        # waiting 13 hours. Anything other than 1.0 is reported in the message
        # so a plot can never be mistaken for a real discharge curve.
        self.declare_parameter('time_scale', 1.0)

        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.capacity_wh = g('capacity_wh')
        self.voltage = g('nominal_voltage')
        self.max_speed = g('max_speed')
        self.max_payload = g('max_payload_kg')
        self.ref_speed = g('reference_speed')
        self.payload = g('payload_kg')
        self.time_scale = g('time_scale')

        self.p_standby, self.p_drive, self.p_payload = self.fit_power(
            self.capacity_wh, g('runtime_standby_h'),
            g('runtime_unloaded_h'), g('runtime_loaded_h'),
            self.ref_speed, self.max_speed)

        self.energy_wh = self.capacity_wh * float(g('initial_soc'))
        self.speed = 0.0
        self.last_t = None

        self.pub = self.create_publisher(BatteryState, 'battery_state', 10)
        self.create_subscription(Odometry, '/diff_drive_controller/odom',
                                 self._odom, 10)
        self.create_timer(1.0 / float(g('publish_rate')), self._tick)

        self.get_logger().info(
            f'battery: {self.capacity_wh:.0f} Wh, fitted standby '
            f'{self.p_standby:.1f} W, drive +{self.p_drive:.1f} W at '
            f'{self.ref_speed} m/s, payload +{self.p_payload:.1f} W at full '
            f'load and {self.ref_speed} m/s'
            + ('' if self.time_scale == 1.0
               else f'  [TIME SCALED x{self.time_scale}, not a real curve]'))

    @staticmethod
    def fit_power(capacity_wh, h_standby, h_unloaded, h_loaded,
                  ref_speed, max_speed):
        """Solve the three published runtimes for three power terms.

        P(v, m) = p_standby
                + p_drive   * (v / max_speed)
                + p_payload * (m / max_payload) * (v / max_speed)

        evaluated at v = ref_speed. Returns the three coefficients such that the
        published runtimes come out exactly at that speed.
        """
        p_standby = capacity_wh / h_standby
        p_active_unloaded = capacity_wh / h_unloaded
        p_active_loaded = capacity_wh / h_loaded

        duty = ref_speed / max_speed
        if duty <= 0.0:
            raise ValueError('reference_speed must be above zero')

        p_drive = (p_active_unloaded - p_standby) / duty
        p_payload = (p_active_loaded - p_active_unloaded) / duty
        return p_standby, p_drive, p_payload

    def power_w(self, speed, payload_kg):
        frac_v = min(abs(speed) / self.max_speed, 1.0)
        frac_m = min(payload_kg / self.max_payload, 1.0) if self.max_payload else 0.0
        return (self.p_standby
                + self.p_drive * frac_v
                + self.p_payload * frac_m * frac_v)

    def _odom(self, msg):
        self.speed = msg.twist.twist.linear.x

    def _tick(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.last_t is None:
            self.last_t = now
            return
        dt = (now - self.last_t) * self.time_scale
        self.last_t = now
        if dt <= 0.0:
            return

        p = self.power_w(self.speed, self.payload)
        self.energy_wh = max(0.0, self.energy_wh - p * dt / 3600.0)

        soc = self.energy_wh / self.capacity_wh if self.capacity_wh else 0.0
        m = BatteryState()
        m.header.stamp = self.get_clock().now().to_msg()
        m.voltage = float(self.voltage)
        m.current = float(-p / self.voltage) if self.voltage else 0.0
        m.charge = float(self.energy_wh / self.voltage) if self.voltage else 0.0
        m.capacity = float(self.capacity_wh / self.voltage) if self.voltage else 0.0
        m.design_capacity = m.capacity
        m.percentage = float(soc)
        m.power_supply_status = (
            BatteryState.POWER_SUPPLY_STATUS_DISCHARGING if p > 0
            else BatteryState.POWER_SUPPLY_STATUS_NOT_CHARGING)
        m.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_GOOD
        m.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LION
        m.present = True
        self.pub.publish(m)


def main():
    rclpy.init()
    node = BatteryModel()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
