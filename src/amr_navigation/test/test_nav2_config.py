#!/usr/bin/env python3
"""Checks on the generated Nav2 configuration.

WHAT THESE ARE FOR. `nav2.yaml` used to be hand-written and carried MiR250
derived literals for the footprint, the speed limits, the inflation radius and
the local costmap size. Nothing checked them against the platform, so with a
second platform a vehicle could be given a correct body and another machine's
navigation tuning, and every file involved would look properly configured.

These run over EVERY platform spec, like the field tests and the spec tests. A
platform that has no generated configuration fails here rather than at launch,
where the symptom is a lifecycle node stuck in unconfigured reporting only
"failed to change state".

They need no ROS and run in milliseconds.
"""

import importlib.util
import math
import subprocess
import sys
from pathlib import Path

import pytest
import yaml

PKG = Path(__file__).resolve().parents[1]
GEN = PKG / 'tools' / 'generate_nav2.py'
TEMPLATE = PKG / 'config' / 'nav2.yaml.in'
SPEC_DIR = (Path(__file__).resolve().parents[2]
            / 'amr_description' / 'config' / 'platforms')


def cfg_path(name):
    return PKG / 'config' / f'nav2.{name}.yaml'


def _load_generator():
    spec = importlib.util.spec_from_file_location('generate_nav2', GEN)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


gen = _load_generator()


@pytest.fixture(params=sorted(p.stem for p in SPEC_DIR.glob('*.yaml')),
                scope='module')
def platform_name(request):
    return request.param


@pytest.fixture(scope='module')
def spec(platform_name):
    return yaml.safe_load((SPEC_DIR / f'{platform_name}.yaml').read_text())


@pytest.fixture(scope='module')
def values(spec):
    return spec['values']


@pytest.fixture(scope='module')
def cfg(platform_name):
    path = cfg_path(platform_name)
    assert path.is_file(), (
        f'no generated Nav2 configuration for platform {platform_name}: run '
        f'tools/generate_nav2.py --platform {platform_name}. Without one the '
        f'launch has nothing to load, and a fallback file would be worse: it '
        f'would hand this vehicle another platform\'s tuning.')
    return yaml.safe_load(path.read_text())


def _costmaps(cfg):
    return {
        'global_costmap': cfg['global_costmap']['global_costmap']['ros__parameters'],
        'local_costmap': cfg['local_costmap']['local_costmap']['ros__parameters'],
    }


def _mppi(cfg):
    return cfg['controller_server']['ros__parameters']['FollowPath']


def _footprint(costmap):
    import ast
    return ast.literal_eval(costmap['footprint'])


def test_generated_file_is_current(platform_name, tmp_path):
    """The config is generated. If someone edits it by hand, say so.

    Compared as TEXT, not as parsed YAML, and the difference matters here. Most
    of this file is comments, and the comments carry the measured failure
    behind each choice. A parsed comparison would let someone quietly rewrite
    the reasoning while the numbers still matched, which in this repository is
    the more damaging edit of the two.
    """
    out = tmp_path / 'regenerated.yaml'
    subprocess.run(
        [sys.executable, str(GEN), '--platform', platform_name,
         '--out', str(out)],
        check=True, capture_output=True)
    assert out.read_text() == cfg_path(platform_name).read_text(), (
        f'config/nav2.{platform_name}.yaml differs from what the generator '
        f'produces; it was hand-edited, or the spec or nav2.yaml.in changed '
        f'without regenerating')


def test_every_platform_has_a_configuration(platform_name):
    """Adding a platform spec must not leave navigation behind.

    This is the same rule the platform spec tests enforce for provenance: a
    spec file is a claim that the platform is supported, and a claim nothing
    backs is worse than an absence.
    """
    assert cfg_path(platform_name).is_file()


def test_the_template_is_the_only_hand_written_source(platform_name):
    """No stray nav2.yaml may come back.

    A file with that name would be loaded by nobody, since the launch selects
    by platform, but it would read as the live configuration to anyone opening
    the directory and would drift silently from the generated ones.
    """
    assert not (PKG / 'config' / 'nav2.yaml').exists(), (
        'config/nav2.yaml is back. The configuration is generated per platform '
        'as nav2.<platform>.yaml; the hand-written source is nav2.yaml.in')
    assert TEMPLATE.is_file()


def test_footprint_is_the_platform_footprint(values, cfg):
    """Both costmaps must describe the vehicle they are driving.

    The footprint is the scanner optical centres rather than the published
    chassis rectangle, because on both platforms the optics sit at the envelope
    corners and 5 mm proud of them, so they are the outermost fixed structure a
    gap has to clear.
    """
    expected = {(values['scanner_mount_x'], values['scanner_mount_y']),
                (values['scanner_mount_x'], -values['scanner_mount_y']),
                (-values['scanner_mount_x'], -values['scanner_mount_y']),
                (-values['scanner_mount_x'], values['scanner_mount_y'])}
    for name, costmap in _costmaps(cfg).items():
        got = {(round(x, 6), round(y, 6)) for x, y in _footprint(costmap)}
        assert got == {(round(x, 6), round(y, 6)) for x, y in expected}, (
            f'{name} footprint {sorted(got)} is not this platform\'s')


def test_the_two_costmaps_agree_about_the_vehicle(cfg):
    """A vehicle that is one size globally and another locally plans nonsense."""
    maps = _costmaps(cfg)
    assert _footprint(maps['global_costmap']) == _footprint(maps['local_costmap'])
    assert (maps['global_costmap']['inflation_layer']['inflation_radius']
            == maps['local_costmap']['inflation_layer']['inflation_radius'])
    assert (maps['global_costmap']['footprint_padding']
            == maps['local_costmap']['footprint_padding'])


def test_inflation_covers_the_whole_vehicle(values, cfg):
    """An inflation radius inside the circumscribed radius inflates nothing.

    Nav2 inflates from the inscribed radius outward. A radius smaller than the
    circumscribed radius leaves the corners of a rectangular vehicle outside
    the inflated band entirely, so the cost gradient that is supposed to keep
    the vehicle off the walls does not cover the parts of it that hit them.
    """
    r_circ = math.hypot(values['scanner_mount_x'], values['scanner_mount_y'])
    for name, costmap in _costmaps(cfg).items():
        radius = costmap['inflation_layer']['inflation_radius']
        assert radius > r_circ, (
            f'{name} inflation radius {radius:.4f} m is inside the '
            f'circumscribed radius {r_circ:.4f} m, so the vehicle\'s corners '
            f'sit outside their own inflation')


def test_speed_limits_stay_inside_the_platform_ratings(values, cfg):
    """Navigation may be slower than the vehicle. It may never be faster."""
    mppi = _mppi(cfg)
    assert 0 < mppi['vx_max'] <= values['max_linear_speed'], (
        f"vx_max {mppi['vx_max']} exceeds the platform's "
        f"{values['max_linear_speed']} m/s rating")
    assert mppi['vx_min'] >= -values['max_reverse_speed'], (
        f"vx_min {mppi['vx_min']} reverses faster than the platform's "
        f"{values['max_reverse_speed']} m/s limit")
    assert 0 < mppi['wz_max'] <= values['max_angular_speed']
    assert mppi['ax_max'] <= values['max_linear_accel_unladen'] + 1e-9


def test_ordinary_braking_stays_inside_the_emergency_reserve(values, cfg):
    """THE MP-400 REGRESSION, and it is not a hypothetical one.

    That platform's unladen acceleration rating is 2.4 m/s2 and its emergency
    deceleration is 1.5 m/s2. Carried across literally, the controller would
    brake harder in ordinary driving than the protective fields assume it can
    in an emergency. Every stopping distance behind those fields is computed
    from the emergency rate, so the vehicle would be routinely exceeding the
    figure its own safety case rests on, and nothing in the stack would notice.
    """
    ax_min = _mppi(cfg)['ax_min']
    assert -ax_min < values['emergency_decel'], (
        f'ordinary braking {-ax_min} m/s2 is not inside the emergency rate '
        f"{values['emergency_decel']} m/s2")


def test_the_smoother_does_not_contradict_the_controller(cfg):
    """The smoother sits between MPPI and the wheels and can only subtract.

    If it permitted more than the controller was allowed, the limits MPPI plans
    against would not be the limits the vehicle drives at, and the trajectory
    that was scored is not the one that runs.
    """
    mppi = _mppi(cfg)
    sm = cfg['velocity_smoother']['ros__parameters']
    assert sm['max_velocity'][0] == pytest.approx(mppi['vx_max'])
    assert sm['min_velocity'][0] == pytest.approx(mppi['vx_min'])
    assert sm['max_velocity'][2] == pytest.approx(mppi['wz_max'])
    assert sm['max_accel'][0] == pytest.approx(mppi['ax_max'])
    assert sm['max_decel'][0] == pytest.approx(mppi['ax_min'])


def test_the_local_costmap_holds_the_controller_horizon(cfg):
    """A window shorter than the horizon is scored against cells with no data.

    MPPI rolls trajectories out for time_steps * model_dt seconds. At the
    commissioned speed that is a distance, and the rolling window has to
    contain it in front of the vehicle, which for a window centred on the
    vehicle means half the width.
    """
    mppi = _mppi(cfg)
    local = _costmaps(cfg)['local_costmap']
    lookahead = mppi['vx_max'] * mppi['time_steps'] * mppi['model_dt']
    assert local['width'] / 2.0 >= lookahead, (
        f"local costmap is {local['width']} m across, so it holds "
        f"{local['width'] / 2.0:.2f} m ahead against a {lookahead:.2f} m "
        f'controller horizon')
    assert local['width'] == local['height']


def test_the_voxel_layer_covers_the_vehicle_envelope(values, cfg):
    """What has to clear is the deck plus its load, not the chassis.

    The camera layer exists because a 2D scan plane is blind to most of what
    should block this vehicle. If its marking band stops below the envelope,
    it is blind to the same things again, higher up.
    """
    envelope = values['vehicle_envelope_height']
    for name, costmap in _costmaps(cfg).items():
        voxel = costmap['voxel_layer']
        assert voxel['z_voxels'] * voxel['z_resolution'] == pytest.approx(
            envelope, abs=1e-6), (
            f'{name} voxel layer spans '
            f"{voxel['z_voxels'] * voxel['z_resolution']:.2f} m against a "
            f'{envelope:.2f} m vehicle envelope')
        for source in voxel['observation_sources'].split():
            assert voxel[source]['max_obstacle_height'] == pytest.approx(envelope)


def test_the_planner_is_not_given_a_turning_radius_it_does_not_have(cfg):
    """Both platforms are differential drives and turn on the spot.

    Recorded as a test rather than a comment because the day a towed cart is
    added this stops being true, and the failure would otherwise be a planner
    quietly refusing manoeuvres the vehicle can make.
    """
    planner = cfg['planner_server']['ros__parameters']['GridBased']
    assert planner['plugin'] == 'nav2_smac_planner::SmacPlanner2D'
    assert _mppi(cfg)['motion_model'] == 'DiffDrive'
    assert _mppi(cfg)['vy_max'] == 0.0


def test_navigation_output_never_reaches_the_wheels_directly(platform_name):
    """The collision monitor is the last thing before the wheels.

    The whole safety argument is the ORDER: nothing in the Nav2 configuration
    may publish onto the controller's command topic, because that would put a
    planner in direct control of a vehicle sharing a floor with people. The
    remapping that enforces it lives in the launch file, so what is asserted
    here is that nothing in the configuration undoes it.
    """
    text = cfg_path(platform_name).read_text()
    assert 'diff_drive_controller/cmd_vel' not in text, (
        'the Nav2 configuration names the controller command topic; the only '
        'thing that may publish there is the collision monitor')


def test_the_ground_truth_map_never_reaches_the_navigation_stack(platform_name):
    """/ground_truth/ is measurement only, and this is a build-failing rule.

    A costmap layer that subscribed to the true floorplan would make every
    mapping and navigation result meaningless while looking like an
    improvement, which is the most expensive kind of mistake available here.
    """
    text = cfg_path(platform_name).read_text()
    for line in text.splitlines():
        stripped = line.strip()
        if stripped.startswith('#'):
            continue          # the comments discuss the rule, and must
        assert '/ground_truth' not in stripped, (
            f'the navigation configuration consumes ground truth: {line!r}')
