#!/usr/bin/env python3
"""Checks on the generated protective and warning fields.

These assert properties a protective field must have. They are not tests of the
collision monitor, which is upstream and tested there; they are tests that the
geometry handed to it is derived correctly and cannot silently drift when the
platform spec changes.
"""

import importlib.util
from pathlib import Path

import pytest
import yaml

PKG = Path(__file__).resolve().parents[1]
GEN = PKG / 'tools' / 'generate_fields.py'
CFG = PKG / 'config' / 'collision_monitor.yaml'
SPEC = (Path(__file__).resolve().parents[2]
        / 'amr_description' / 'config' / 'platforms' / 'mir250_class.yaml')
SPEC_ALL = yaml.safe_load(SPEC.read_text())


def _load_generator():
    spec = importlib.util.spec_from_file_location('generate_fields', GEN)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


gen = _load_generator()


@pytest.fixture(scope='module')
def platform():
    return yaml.safe_load(SPEC.read_text())['values']


@pytest.fixture(scope='module')
def cfg():
    return yaml.safe_load(CFG.read_text())['collision_monitor']['ros__parameters']


def _polys(cfg, group):
    block = cfg[group]
    return {name: block[name] for name in block['velocity_polygons']}


def _points(poly):
    """Nav2 carries polygon points as a string, so parse them back."""
    import ast
    return ast.literal_eval(poly['points'])


def _is_rotation(name):
    """Rotation bands are the ones pinned to near-zero linear velocity."""
    return name.split('_', 1)[1].startswith('rot_')


def _rotation_bands(cfg, group):
    return {n: p for n, p in _polys(cfg, group).items() if _is_rotation(n)}


def _reach(poly):
    """How far the field extends ahead of the robot centre."""
    return max(x for x, _ in _points(poly))


def _rear_reach(poly):
    """How far the field extends BEHIND the robot centre, as a positive number."""
    return -min(x for x, _ in _points(poly))


def _directed_reach(poly):
    """Reach in the direction the band's own velocities point.

    A reverse field extends backward, so measuring it as maximum x reports the
    front face and reads as though the field had no depth at all. That is
    exactly what happened when reverse and spot-turn bands were added: the
    warning-encloses-protective test compared 0.400 m against 0.400 m, both of
    them the front face of a field that reaches backward.
    """
    if poly['linear_max'] <= 0.0:
        return _rear_reach(poly)
    return _reach(poly)


def test_stopping_distance_matches_the_hand_calculation(platform):
    v = 2.0
    expected = (
        v * (platform['scanner_response_time'] + platform['control_latency']
             + platform['brake_actuation_delay'])
        + v * v / (2.0 * platform['emergency_decel'])
        + platform['scanner_protective_supplement'])
    assert gen.stopping_distance(v, platform) == pytest.approx(expected)


def test_stopping_distance_grows_faster_than_speed(platform):
    """Braking is quadratic, so doubling the speed more than doubles the field.

    This is why a single fixed protective field cannot serve a vehicle with a
    wide speed range, and therefore why the fields are velocity switched.
    """
    a = gen.stopping_distance(1.0, platform)
    b = gen.stopping_distance(2.0, platform)
    assert b > 2.0 * a


def test_zero_speed_still_leaves_the_supplement(platform):
    """Even stationary, the scanner cannot resolve an object at the boundary."""
    assert gen.stopping_distance(0.0, platform) == pytest.approx(
        platform['scanner_protective_supplement'])


def test_every_protective_field_covers_its_stopping_distance(platform, cfg):
    """The property that makes it a protective field rather than a decoration.

    For each speed band, the field must reach at least the stopping distance at
    the TOP of that band, measured from the front face.
    """
    half_length = platform['chassis_length'] / 2.0
    for name, poly in _polys(cfg, 'protective').items():
        if _is_rotation(name):
            continue          # covered by their own tests; there is no linear
                              # speed to derive a stopping distance from
        speed = max(abs(poly['linear_max']), abs(poly['linear_min']))
        needed = gen.stopping_distance(speed, platform)
        have = _directed_reach(poly) - half_length
        assert have >= needed - 1e-6, (
            f'{name} reaches {have:.3f} m beyond the face it protects but needs '
            f'{needed:.3f} m to stop from {speed} m/s')


def test_spot_turn_field_covers_the_swept_corner(platform, cfg):
    """A rotating vehicle has no linear speed, but its corners still move.

    The corner furthest from the turn centre travels at w * circumscribed
    radius, and that is the speed the all-round field has to stop from. Before
    this band existed, a turn faster than 1 rad/s matched no polygon at all and
    the monitor went silent, which stops the vehicle by accident rather than by
    design.
    """
    w_max = platform['max_angular_speed']
    poly = next(p for p in _rotation_bands(cfg, 'protective').values()
                if p['theta_max'] == pytest.approx(w_max))
    r_circ = (platform['chassis_length'] ** 2 / 4.0
              + platform['chassis_width'] ** 2 / 4.0) ** 0.5
    needed = gen.stopping_distance(platform['max_angular_speed'] * r_circ, platform)
    pts = _points(poly)
    half_length = platform['chassis_length'] / 2.0
    half_width = platform['chassis_width'] / 2.0
    assert min(x for x, _ in pts) <= -(half_length + needed) + 1e-6
    assert max(x for x, _ in pts) >= (half_length + needed) - 1e-6
    assert min(y for _, y in pts) <= -(half_width + needed) + 1e-6
    assert max(y for _, y in pts) >= (half_width + needed) - 1e-6


def test_warning_field_always_encloses_the_protective_field(cfg):
    """Otherwise the vehicle would stop before it ever slowed down."""
    stop = _polys(cfg, 'protective')
    warn = _polys(cfg, 'warning')
    for name, s in stop.items():
        w = warn[name.replace('stop_', 'warn_', 1)]
        assert _directed_reach(w) > _directed_reach(s), (
            f'{name}: warning field reaches {_directed_reach(w):.3f} m, '
            f'protective {_directed_reach(s):.3f} m; the slowdown would '
            f'never fire')


def test_no_commandable_velocity_falls_outside_every_band(platform, cfg):
    """A velocity that matches no polygon SILENCES the monitor.

    This is the failure mode that matters most and it is the least obvious. The
    monitor does not pass an uncovered command through, and it does not stop the
    vehicle either: it publishes nothing at all, so the wheels receive no
    command while the controller upstream carries on publishing at 20 Hz. It was
    found only by noticing that /diff_drive_controller/cmd_vel had no publisher
    while /cmd_vel_raw had one, with the monitor logging "Velocity is not
    covered by any of the velocity polygons. x: -0.150".

    So this walks the whole commandable velocity space, forward and reverse,
    across the full angular range, and asserts every point matches some band.
    """
    polys = list(_polys(cfg, 'protective').values())
    v_max = platform['max_linear_speed']
    v_rev = platform['max_reverse_speed']
    w_max = platform['max_angular_speed']

    uncovered = []
    steps = 41
    for i in range(steps):
        vx = -v_rev + (v_max + v_rev) * i / (steps - 1)
        for j in range(steps):
            wz = -w_max + 2 * w_max * j / (steps - 1)
            if not any(p['linear_min'] <= vx <= p['linear_max']
                       and p['theta_min'] <= wz <= p['theta_max']
                       for p in polys):
                uncovered.append((round(vx, 3), round(wz, 3)))
    assert not uncovered, (
        f'{len(uncovered)} commandable velocities match no protective field, '
        f'so the monitor would go silent at them. First few: {uncovered[:5]}')


def test_forward_speed_bands_are_contiguous(platform, cfg):
    """A gap between forward bands is a speed at which no field applies."""
    polys = sorted((p for p in _polys(cfg, 'protective').values()
                    if p['linear_max'] > 0.05),
                   key=lambda p: p['linear_max'])
    for a, b in zip(polys, polys[1:]):
        assert b['linear_min'] == pytest.approx(a['linear_max']), (
            f'gap between {a["linear_max"]} and {b["linear_min"]} m/s')
    assert polys[-1]['linear_max'] == pytest.approx(platform['max_linear_speed']), (
        'the fastest band does not reach the platform top speed, so the vehicle '
        'could travel at a speed no field covers')


def test_linear_bands_cover_the_full_angular_range(platform, cfg):
    """A vehicle driving forward may also be turning at its maximum rate.

    The linear bands previously declared plus or minus 1.0 rad/s against a
    platform capable of 1.5, so a fast turn while driving matched no band. The
    rotation bands are excluded here because dividing the angular range between
    them is the entire point of them.
    """
    w_max = platform['max_angular_speed']
    for group in ('protective', 'warning'):
        for name, poly in _polys(cfg, group).items():
            if _is_rotation(name):
                continue
            assert poly['theta_min'] <= -w_max + 1e-9, name
            assert poly['theta_max'] >= w_max - 1e-9, name


def test_creep_field_fits_the_dynamic_corridor(platform, cfg):
    """THE DEADLOCK REGRESSION TEST.

    A single all-round field covering every rotation rate reached 0.766 m to
    each side, wider than the half width of the 1.00 m dynamic corridor the
    vehicle claims to work in. The effect was not a slow robot but a stuck one:
    starting from rest the controller commands a few millimetres per second,
    that selected the all-round field, the field was inside the racking, the
    monitor held the vehicle stopped, and it never reached a speed that would
    have selected a narrower field. Measured, 0.011 m/s commanded for three
    minutes and 0.28 m travelled.

    So the field that applies while creeping MUST fit the corridor, or the
    vehicle cannot start.
    """
    corridor_half = SPEC_ALL['validation_targets']['corridor_width_dynamic'] / 2.0
    # The band that applies while creeping essentially straight, which is the
    # state every journey has to pass through on its way out of rest.
    poly = next(p for p in _rotation_bands(cfg, 'protective').values()
                if p['theta_min'] <= 0.0 <= p['theta_max'])
    widest = max(abs(y) for _, y in _points(poly))
    assert widest <= corridor_half + 1e-9, (
        f'the creeping protective field is {widest:.3f} m half width against a '
        f'corridor half width of {corridor_half:.3f} m, so the vehicle would be '
        f'held stopped at the speed it has to pass through to start moving')


def test_rotation_bands_tile_the_angular_range(platform, cfg):
    """Split into creep and hard turn, they must still leave no gap."""
    w_max = platform['max_angular_speed']
    for group in ('protective', 'warning'):
        bands = list(_rotation_bands(cfg, group).values())
        for k in range(201):
            wz = -w_max + 2 * w_max * k / 200
            assert any(b['theta_min'] <= wz <= b['theta_max'] for b in bands), (
                f'{group}: rotating at {wz:.3f} rad/s with no linear speed '
                f'matches no band')


def test_fields_are_at_least_as_wide_as_the_vehicle(platform, cfg):
    """A field narrower than the robot lets it drive its own corner into
    something the field never covered."""
    half_width = platform['chassis_width'] / 2.0
    for group in ('protective', 'warning'):
        for name, poly in _polys(cfg, group).items():
            widest = max(abs(y) for _, y in _points(poly))
            assert widest >= half_width, (
                f'{name} is {widest:.3f} m half-width against a vehicle '
                f'half-width of {half_width:.3f} m')


def test_polygon_points_are_the_string_form_nav2_requires(cfg):
    """Nav2 rejects a numeric array here, and the failure is opaque.

    Configuring with a double array fails with 'parameter points has invalid
    type', and the lifecycle manager reports only 'failed to change state'. The
    node had to be run standalone to see the real message.
    """
    for group in ('protective', 'warning'):
        for name, poly in _polys(cfg, group).items():
            assert isinstance(poly['points'], str), (
                f'{name} points is {type(poly["points"]).__name__}; nav2 needs a string')
            parsed = _points(poly)
            assert len(parsed) >= 3, f'{name} is not a polygon'
            assert all(len(pt) == 2 for pt in parsed)


def test_command_output_is_stamped(cfg):
    """The controller takes TwistStamped only, and the mismatch is silent.

    With plain Twist, both message types end up advertised on the same topic,
    diff_drive_controller ignores the one it does not want, and the robot never
    moves with no error logged anywhere. It presented as a permanent protective
    stop.
    """
    assert cfg['enable_stamped_cmd_vel'] is True


def test_the_source_is_the_merged_scan_and_not_the_classifier(cfg):
    """The architectural rule, asserted so it cannot be quietly changed.

    A protective function must not depend on classification. The people
    detector measures about 0.18 precision, while raw returns from a pedestrian
    are present in 100 percent of frames. Wiring the classifier in here would
    make safety depend on the weaker signal.
    """
    sources = cfg['observation_sources']
    assert sources == ['merged_scan']
    assert cfg['merged_scan']['topic'] == 'scan'
    for name in sources:
        topic = cfg[name]['topic']
        assert 'detection' not in topic and 'track' not in topic and 'people' not in topic, (
            f'observation source {name} consumes {topic}, which is a '
            f'classifier output; safety must not depend on it')


def test_generated_file_is_current(platform, tmp_path):
    """The config is generated. If someone edits it by hand, say so."""
    out = tmp_path / 'regenerated.yaml'
    import subprocess
    import sys
    subprocess.run(
        [sys.executable, str(GEN), '--out', str(out)],
        check=True, capture_output=True)
    fresh = yaml.safe_load(out.read_text())
    current = yaml.safe_load(CFG.read_text())
    assert fresh == current, (
        'config/collision_monitor.yaml differs from what the generator '
        'produces; it was hand-edited, or the spec changed without regenerating')


def test_starting_from_rest_is_not_self_blocking(platform, cfg):
    """THE DEADLOCK CLASS, asserted directly rather than by proxy.

    Both deadlocks so far had the same shape. The vehicle starts from rest. The
    controller is acceleration limited, so its first command is one step above
    zero, a few millimetres per second and barely turning. If the field that
    applies at that command is bigger, in the directions that matter, than the
    field that applies once moving, the vehicle can be held stopped by something
    it is not driving towards and can never reach a speed that would give it a
    smaller field. It is stuck in a state it must pass through to become
    unstuck. Measured on the running system: 0.10 m travelled in 45 seconds,
    with the stop caused by a rack corner 0.66 m BEHIND the vehicle.

    The property is directional, not a matter of total area. A rotating vehicle
    legitimately needs clearance behind it that a forward-driving one does not,
    so comparing areas would forbid a correct field. What must hold is that
    crawling out of rest is not more restricted SIDEWAYS or AHEAD than cruising,
    because those are the directions an aisle constrains and the direction of
    travel respectively.
    """
    at_rest = next(p for p in _rotation_bands(cfg, 'protective').values()
                   if p['theta_min'] <= 0.0 <= p['theta_max'])
    cruising = next(p for n, p in _polys(cfg, 'protective').items()
                    if not _is_rotation(n)
                    and p['linear_min'] <= 0.20 <= p['linear_max'])

    rest_pts, cruise_pts = _points(at_rest), _points(cruising)

    # SIDEWAYS is compared against the CORRIDOR, not against the cruising
    # field. Crawling out of rest is legitimately a fraction wider than
    # cruising, by exactly the distance the corners sweep, and demanding
    # otherwise would forbid a physically correct allowance. What matters is
    # that the field still fits the aisle the vehicle claims to work in.
    corridor_half = SPEC_ALL['validation_targets']['corridor_width_dynamic'] / 2.0
    rest_side = max(abs(y) for _, y in rest_pts)
    assert rest_side <= corridor_half + 1e-9, (
        f'crawling out of rest gives a field {rest_side:.3f} m to each side in '
        f'a corridor {corridor_half * 2:.2f} m wide, so the walls hold the '
        f'vehicle stopped at the speed it must pass through to start moving')

    rest_ahead = max(x for x, _ in rest_pts)
    cruise_ahead = max(x for x, _ in cruise_pts)
    assert rest_ahead <= cruise_ahead + 1e-9, (
        f'crawling out of rest looks {rest_ahead:.3f} m ahead against '
        f'{cruise_ahead:.3f} m once moving, so an obstacle in the direction of '
        f'travel blocks the start but not the journey')


def test_rotation_fields_do_not_double_count_the_supplement(platform, cfg):
    """The all-round fields grow the RAW footprint, not the supplemented one.

    A stopping distance already contains the scanner's protective field
    supplement. The forward fields add it laterally on their own, because their
    sides are not a stopping distance. Adding a reach that contains it to a half
    width that also contains it applied it twice and made the near-straight
    creep field 0.432 m half width instead of 0.367 m, a third too wide on the
    dimension that decides whether the vehicle fits an aisle.
    """
    half_width_raw = platform['chassis_width'] / 2.0
    r_circ = (platform['chassis_length'] ** 2 / 4.0
              + platform['chassis_width'] ** 2 / 4.0) ** 0.5
    for name, poly in _rotation_bands(cfg, 'protective').items():
        w_ref = max(abs(poly['theta_min']), abs(poly['theta_max']))
        expected = half_width_raw + gen.stopping_distance(w_ref * r_circ, platform)
        widest = max(abs(y) for _, y in _points(poly))
        assert widest == pytest.approx(expected, abs=2e-4), (
            f'{name} is {widest:.4f} m half width, expected {expected:.4f} m; '
            f'the supplement is being applied twice')


def test_the_warning_field_limits_speed_rather_than_scaling_it(platform, cfg):
    """A multiplicative slowdown is a stable trap, not a slowdown.

    Downstream of an acceleration-limited controller closing its loop on
    measured velocity, scaling the command by a ratio r settles at
    v = r*a*dt/(1-r), independent of what the controller wanted. With the
    ratio of 0.3 that was configured here that is 0.0064 m/s, and the vehicle
    covered 0.10 m in 45 seconds while every component reported healthy.

    Capping the speed instead lets the controller ramp to the cap and hold it.
    """
    warning = cfg['warning']
    assert warning['action_type'] == 'limit', (
        'the warning field scales the command instead of capping the speed, '
        'which throttles an acceleration-limited controller to a standstill')
    assert 'slowdown_ratio' not in warning
    assert warning['linear_limit'] > 0.0
    assert warning['angular_limit'] > 0.0
    # The cap has to be a speed the vehicle can actually work at, which means
    # its own protective field must fit the aisle.
    fits = [p for n, p in _polys(cfg, 'protective').items()
            if not _is_rotation(n) and p['linear_min'] <= warning['linear_limit']
            <= p['linear_max']]
    assert fits, 'the warning speed cap falls outside every protective band'
