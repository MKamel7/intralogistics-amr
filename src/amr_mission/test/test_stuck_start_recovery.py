"""The recovery for a planner that refuses to start. V-58.

WHAT THE DEFECT WAS

    GridBased plugin failed to plan from (4.68, 5.68) to (-2.00, 0.00):
      "Start occupied"

Three cycles, three failures, three seconds each, 0.0 m driven. The vehicle
finished its survey in a pose whose own cell reads as occupied, and from there
the planner refuses, so no command reaches the wheels, so the vehicle does not
move, so the start stays occupied. The mission retried the same goal from the
same pose and failed identically.

WHY THE GUARD IS A DISTANCE AND NOT AN ERROR STRING

Matching on "Start occupied" would tie the mission to one planner's wording.
SmacPlanner2D says that; NavFn says "Failed to create plan with tolerance";
ThetaStar says "Either of the start or goal pose are an obstacle" (V-47). A leg
that failed having driven nothing is the observable common to all three and it
does not care what the planner called it.

WHY NOT CLEAR THE FOOTPRINT FROM THE COSTMAP

That is the obvious fix and it trades a stall for a collision risk. V-42 and
V-45 are what this project has to show for safety changes argued from
arithmetic rather than measured, and both were reverted.
"""

import re
from pathlib import Path

SRC = Path(__file__).resolve().parents[1] / 'amr_mission' / 'transport_task.py'


def source():
    return SRC.read_text()


def code(text):
    """The same text with comment lines removed.

    Every text assertion in this file has to run over this rather than over the
    raw source, and the reason is a mistake made three times in one session: a
    test forbidding `ros_gz_sim` matched the COMMENT explaining why
    `ros_gz_sim` is not used. The file documents its own defects, so a search
    for a defect will find the documentation of it.

    Docstrings are left alone, because stripping them needs a parser and the
    assertions here are about calls rather than prose.
    """
    return '\n'.join(ln for ln in text.splitlines()
                      if not ln.strip().startswith('#'))


def test_a_leg_that_moved_is_not_nudged():
    """The bound that keeps this from firing on ordinary navigation failures.

    A leg that drove and then failed is one where the planner engaged. Whatever
    went wrong is not a refusal to start, and reversing into a corridor the
    vehicle was already driving down would be the wrong answer to it.
    """
    t = source()
    m = re.search(r'if not arrived and \(self\.odom_total - leg_start\) < ([\d.]+):', t)
    assert m, 'the nudge is not gated on the distance the leg drove'
    threshold = float(m.group(1))
    assert threshold <= 0.10, (
        f'the nudge fires on legs that drove up to {threshold} m, which is '
        f'ordinary navigation failure rather than a refusal to start')


def test_the_nudge_reports_whether_it_moved():
    """A nudge that did not move the vehicle has not changed the condition.

    Retrying after one would be the same failure with an extra step, and the
    log would show a recovery that "ran" before every failure.
    """
    t = source()
    assert 'moved = self.odom_total - before' in t
    assert 'return moved > 0.01' in t, (
        'nudge() reports success without checking the vehicle moved')


def test_the_nudge_count_is_always_reported():
    """Zero included.

    "No nudges" and "nudges not counted" look identical in a log otherwise, and
    a recovery that fires constantly is a different problem wearing a solution.
    """
    t = source()
    assert 'self.nudges += 1' in t
    assert re.search(r'nudged out of a stuck start \{self\.nudges\}', t), (
        'the nudge count is not printed in the run summary')


def test_it_uses_the_configured_backup_behaviour():
    """Not a raw cmd_vel.

    Publishing a reverse command directly would bypass the collision monitor's
    command chain. Going through the BackUp action keeps the monitor in the
    loop, and stop_reverse is the one polygon with real rearward margin,
    0.4560 m against a chassis half length of 0.2950 m.
    """
    t = source()
    assert 'from nav2_msgs.action import BackUp' in t
    assert "ActionClient(self, BackUp, 'backup')" in t
    m = re.search(r'goal\.target = Point\(x=([\d.]+)\)', t)
    assert m, 'the nudge distance is not stated'
    assert float(m.group(1)) <= 0.30, (
        f'a {m.group(1)} m reverse is beyond the rearward margin the reverse '
        f'protective field carries')


# ---------------------------------------------------------------------------
# The load as a body rather than as a number.

def test_the_physical_load_is_off_by_default():
    """Every figure measured before this existed must stay comparable.

    A run that does not ask for a physical load has to be exactly the run it
    was, or the braking figures in V-60 stop meaning what they said.
    """
    t = source()
    assert "declare_parameter('physical_load', False)" in t


def test_the_load_is_placed_from_localisation_not_from_ground_truth():
    """A real forklift places a box where it BELIEVES itself to be.

    Using the oracle would make the load's position depend on something the
    vehicle cannot know, and would quietly remove the localisation error from
    a measurement about load handling. /ground_truth/ is measurement only and
    must never reach the control path.
    """
    t = source()
    assert "lookup_transform(\n                'map', 'base_link'" in t, (
        'the load is not placed from the map to base_link transform')
    body = code(t[t.index('def spawn_load'):t.index('def remove_load')])
    assert 'ground_truth' not in body, (
        'the load is placed from the ground truth oracle, which the vehicle '
        'cannot know and which must not reach anything but measurement')


def test_the_load_rests_rather_than_being_welded():
    """The whole point of the physical load.

    A fixed joint carries the mass, which is what V-60 measured, and makes
    whether the load stays on unanswerable by construction.
    """
    model = (SRC.parents[2] / 'amr_sim' / 'models' / 'payload_klt' / 'model.sdf')
    assert model.is_file(), 'the payload model is missing'
    sdf = model.read_text()
    assert '<mu>0.35</mu>' in sdf, 'the friction coefficient is not stated'
    assert '<joint' not in sdf, (
        'the payload model contains a joint, so it is attached to something '
        'and cannot slide off')
    assert '<mass>100.0</mass>' in sdf


def test_the_load_is_placed_above_the_plate_not_inside_it():
    """Interpenetration resolves as a launch.

    A box spawned at exactly the plate height starts intersecting the deck by
    half its own thickness and the solver throws it.
    """
    t = source()
    assert 'self.plate_height + 0.101' in t, (
        'the load is not placed clear of the deck by its own half height plus '
        'a margin')


def test_the_two_gazebo_tools_are_called_with_their_own_conventions():
    """`create` is gflags and `set_entity_pose` is CLI11, in the same package.

    `create` takes -name and -x -y -z; `set_entity_pose` takes --name and
    --pos with three floats and has no world option at all. Passing one
    style to the other fails per cycle and leaves the load on the vehicle,
    which looks exactly like a delivery that did not happen for a navigation
    reason.
    """
    t = source()
    spawn = code(t[t.index('def spawn_load'):t.index('def remove_load')])
    assert "'-name', name" in spawn and "'-world', self.world" in spawn, (
        'create is not called with its gflags style options')
    setdown = code(t[t.index('def remove_load'):t.index('def gz_call')])
    assert 'set_pose' in setdown and 'gz_call' in setdown, (
        'the set down no longer goes through the gz service')
    assert 'ros_gz_sim' not in setdown, (
        'set_entity_pose from ros_gz_sim hangs and killed a run; the service '
        'under it answers in milliseconds')


def test_a_delivered_load_is_set_down_rather_than_deleted():
    """A transport task whose cargo vanishes on arrival has not delivered it.

    Deleting the box makes a run with a load look identical to a run without
    one, which is the same failure as a metric that cannot tell the two arms
    apart.
    """
    t = source()
    assert 'set_entity_pose' in t
    body = code(t[t.index('def remove_load'):t.index('def gz_call')])
    assert 'remove' not in body.replace('remove_load', ''), (
        'the delivered load is still being deleted')


# ---------------------------------------------------------------------------
# Frames. The vehicle drives in the map frame and the simulator speaks world.

def test_the_load_is_placed_in_world_coordinates():
    """The failure this test exists for was silent and total.

    A box asked for at map (-2.05, -0.07) was created at world (-2.05, -0.07),
    which on this track is outside the building. It dropped to the floor, the
    vehicle drove to dispatch carrying nothing, and the mission log said
    "placed payload_0 on the plate". Only the model's own pose gave it away:
    z = 0.0999, which is a 200 mm box resting on the ground.

    Every goal in this file is in the map frame because the vehicle drives to
    it. The spawn service takes world. The two differ by the spawn pose.
    """
    t = source()
    assert 'def map_to_world' in t, (
        'there is no map to world conversion, so a pose meant for the '
        'simulator is being sent in the frame the vehicle navigates in')
    spawn = t[t.index('def spawn_load'):t.index('def remove_load')]
    assert 'self.map_to_world(*pose)' in spawn, (
        'spawn_load places the box without converting to world coordinates')


def test_a_rotated_spawn_is_refused_rather_than_approximated():
    """map to world is a translation only while the spawn yaw is zero.

    With a rotated spawn the same code produces a small, plausible offset
    rather than an obvious failure, which is the harder kind to notice.
    """
    t = source()
    assert "abs(float(self.spawn_world.get('yaw', 0.0))) > 1e-6" in t
    assert 'raise SystemExit' in t[t.index('spawn_world'):t.index('def ', t.index('spawn_world'))]


def test_the_setdown_pose_is_already_world_and_not_converted_twice():
    """The generator writes it in world coordinates and says so in the file.

    Converting it again would move every delivery by the spawn offset, which
    on this track is 4.5 m east and would put the table outside the wall.
    """
    t = source()
    setdown = t[t.index('def remove_load'):t.index('def map_to_world')]
    assert 'map_to_world' not in setdown, (
        'the set down pose is being converted, but the stations file already '
        'records it in world coordinates')


def test_a_failing_cargo_tool_cannot_kill_the_mission():
    """It already did, once, and the cost was the whole run.

    ros_gz_sim's set_entity_pose hangs. Inside the mission that raised
    TimeoutExpired after 30 seconds and took the process down at the first
    delivery, losing three cycles of navigation data to a cargo tool.

    Load handling is something the mission DOES, not something it depends on.
    Every simulator call degrades to a warning.
    """
    t = source()
    assert 'def gz_call' in t
    call = t[t.index('def gz_call'):t.index('def map_to_world')]
    assert 'except (subprocess.TimeoutExpired, OSError)' in call, (
        'a simulator call can still raise out of the load handling')
    assert 'return False' in call
    spawn = t[t.index('def spawn_load'):t.index('def remove_load')]
    assert 'except (subprocess.TimeoutExpired, OSError)' in spawn, (
        'the spawn can still raise and abort the run')
