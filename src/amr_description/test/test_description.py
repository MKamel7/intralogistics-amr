#!/usr/bin/env python3
"""Structural checks on the generated robot description.

These build the URDF from the xacro and assert properties the description must
hold no matter how the platform spec is edited. They exist because two classes
of bug already happened in this project's lineage and both were silent:

  * a mass that quietly stopped matching the published total, because a
    component was added to the description but not to the accounting, and
  * inertia tensors that violate the triangle inequality, which Gazebo Classic
    accepted and Harmonic rightly rejects.

Requires xacro, so it runs as part of the colcon test rather than the bare
pytest gate.
"""

import math
import shutil
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
import yaml

PKG = Path(__file__).resolve().parents[1]
SPEC = PKG / 'config' / 'platforms' / 'mir250_class.yaml'
XACRO = PKG / 'urdf' / 'amr.urdf.xacro'

pytestmark = pytest.mark.skipif(
    shutil.which('xacro') is None, reason='xacro not on PATH')


@pytest.fixture(scope='module')
def spec():
    return yaml.safe_load(SPEC.read_text())


@pytest.fixture(scope='module')
def urdf():
    out = subprocess.run(
        ['xacro', str(XACRO), 'platform:=mir250_class'],
        capture_output=True, text=True, check=True)
    assert not out.stderr.strip(), (
        f'xacro emitted warnings, which must not be ignored:\n{out.stderr}')
    return ET.fromstring(out.stdout)


def _links(urdf):
    return {link.get('name'): link for link in urdf.findall('link')}


def _joints(urdf):
    return {j.get('name'): j for j in urdf.findall('joint')}


def test_total_mass_equals_the_published_figure(urdf, spec):
    """The robot must weigh what the data sheet says it weighs.

    The chassis mass is deliberately computed as the published total minus every
    modelled component, so this passes only if the accounting includes every
    link that carries mass. Add a link with inertia and forget the accounting,
    and this fails.
    """
    v = spec['values']
    expected = v['tare_mass'] + v['battery_mass']
    total = sum(float(link.find('inertial/mass').get('value'))
                for link in urdf.findall('link')
                if link.find('inertial/mass') is not None)
    assert total == pytest.approx(expected, abs=1e-6), (
        f'modelled mass {total:.4f} kg does not equal the published '
        f'{expected} kg. A link with inertia is missing from the mass '
        f'accounting in amr.urdf.xacro.')


def test_every_inertia_is_physically_realisable(urdf):
    bad = []
    for link in urdf.findall('link'):
        inertia = link.find('inertial/inertia')
        if inertia is None:
            continue
        ixx, iyy, izz = (float(inertia.get(k)) for k in ('ixx', 'iyy', 'izz'))
        if min(ixx, iyy, izz) <= 0:
            bad.append(f'{link.get("name")}: non-positive principal moment')
        elif not (ixx + iyy >= izz and ixx + izz >= iyy and iyy + izz >= ixx):
            bad.append(f'{link.get("name")}: triangle inequality violated '
                       f'({ixx:.6g}, {iyy:.6g}, {izz:.6g})')
    assert not bad, 'unrealisable inertia tensors:\n  ' + '\n  '.join(bad)


def test_required_frames_exist(urdf):
    """Frames the rest of the stack binds to by name."""
    required = {
        'base_link', 'base_footprint', 'load_deck',
        'drive_left_wheel', 'drive_right_wheel',
        'scanner_front_left_link', 'scanner_rear_right_link',
        'camera_left_link', 'camera_right_link',
        'camera_left_optical_frame', 'camera_right_optical_frame',
        'imu_link',
    }
    missing = sorted(required - set(_links(urdf)))
    assert not missing, f'frames the stack depends on are missing: {missing}'


def test_all_four_casters_are_real_two_dof_joints(urdf):
    """A caster that cannot swivel is a skid, which is what the predecessor had."""
    joints = _joints(urdf)
    for loc in ('front_left', 'front_right', 'rear_left', 'rear_right'):
        swivel = joints.get(f'caster_{loc}_swivel_joint')
        wheel = joints.get(f'caster_{loc}_wheel_joint')
        assert swivel is not None and wheel is not None, f'caster {loc} incomplete'
        assert swivel.get('type') == 'continuous', f'caster {loc} swivel is fixed'
        assert wheel.get('type') == 'continuous', f'caster {loc} wheel is fixed'
        assert swivel.find('axis').get('xyz').split() == ['0', '0', '1'], (
            f'caster {loc} must swivel about the vertical axis')


def test_wheels_reach_the_ground(urdf, spec):
    """Drive wheels must protrude below the chassis, or the robot rests on its belly."""
    v = spec['values']
    joints = _joints(urdf)
    for side in ('left', 'right'):
        z = float(joints[f'drive_{side}_wheel_joint'].find('origin').get('xyz').split()[2])
        assert z == pytest.approx(v['drive_wheel_radius']), (
            f'drive {side} wheel centre at {z} m does not put its contact patch '
            f'on the ground plane')
    assert v['drive_wheel_radius'] > v['ground_clearance'], (
        'chassis underside sits below the wheel contact patch')


def test_scanners_are_diagonally_opposite(urdf, spec):
    """Two 275 degree scanners only cover a full turn if they are on a diagonal.

    Same corner, or the same side, leaves a blind sector behind the robot, and
    the safety concept in a later phase depends on there not being one.
    """
    joints = _joints(urdf)
    a = [float(x) for x in joints['scanner_front_left_joint'].find('origin').get('xyz').split()]
    b = [float(x) for x in joints['scanner_rear_right_joint'].find('origin').get('xyz').split()]
    assert a[0] * b[0] < 0, 'scanners are on the same end of the robot'
    assert a[1] * b[1] < 0, 'scanners are on the same side of the robot'
    assert a[2] == pytest.approx(b[2]), 'scan planes are at different heights'


def test_scanner_apertures_match_the_spec(urdf, spec):
    v = spec['values']
    found = 0
    for sensor in urdf.iter('sensor'):
        if sensor.get('type') != 'gpu_lidar' or not sensor.get('name').startswith('scanner_'):
            continue
        found += 1
        h = sensor.find('lidar/scan/horizontal')
        aperture = math.degrees(float(h.findtext('max_angle')) - float(h.findtext('min_angle')))
        assert aperture == pytest.approx(v['scanner_scan_angle'], abs=1e-6), (
            f'{sensor.get("name")} aperture {aperture:.2f} deg does not match the '
            f'specified {v["scanner_scan_angle"]} deg')
        assert int(h.findtext('samples')) == v['scanner_samples']
        rng = sensor.find('lidar/range')
        assert float(rng.findtext('max')) == pytest.approx(v['scanner_measuring_range'])
    assert found == 2, f'expected 2 safety scanners in the description, found {found}'


def test_scan_rate_does_not_outrun_the_response_time(urdf, spec):
    """A protective device cannot report faster than it responds.

    Overstating this would flatter every stopping-distance result computed from
    the model, so it is asserted against the sensor rather than trusted.
    """
    v = spec['values']
    limit = 1.0 / v['scanner_response_time']
    for sensor in urdf.iter('sensor'):
        if sensor.get('name', '').startswith('scanner_'):
            rate = float(sensor.findtext('update_rate'))
            assert rate <= limit + 1e-6, (
                f'{sensor.get("name")} publishes at {rate} Hz but the reference '
                f'part responds in {v["scanner_response_time"]} s, a limit of '
                f'{limit:.1f} Hz')


def test_drive_joints_are_the_only_actuated_ones(urdf):
    """Casters are passive. Declaring them in ros2_control would imply otherwise."""
    control = urdf.find('ros2_control')
    assert control is not None, 'no ros2_control block in the description'
    names = {j.get('name') for j in control.findall('joint')}
    assert names == {'drive_left_wheel_joint', 'drive_right_wheel_joint'}, (
        f'ros2_control should expose exactly the two drive joints, got {sorted(names)}')


def test_wheel_command_limit_follows_the_platform_top_speed(urdf, spec):
    v = spec['values']
    expected = v['max_linear_speed'] / v['drive_wheel_radius']
    control = urdf.find('ros2_control')
    for joint in control.findall('joint'):
        cmd = joint.find("command_interface[@name='velocity']")
        hi = float(cmd.find("param[@name='max']").text)
        assert hi == pytest.approx(expected), (
            f'{joint.get("name")} velocity limit {hi} rad/s does not match '
            f'{v["max_linear_speed"]} m/s over a {v["drive_wheel_radius"]} m wheel '
            f'({expected:.2f} rad/s)')


def test_controllers_yaml_matches_the_platform_spec(spec):
    """The controller config duplicates kinematics that live in the spec.

    controller_manager reads plain YAML and cannot resolve the spec itself, so
    the values are duplicated on purpose. This is the gate that stops the two
    drifting apart, which would show up as odometry error nobody could explain.
    """
    cfg = yaml.safe_load((PKG / 'config' / 'controllers.yaml').read_text())
    dd = cfg['diff_drive_controller']['ros__parameters']
    v = spec['values']

    assert dd['wheel_separation'] == pytest.approx(v['wheel_separation'])
    assert dd['wheel_radius'] == pytest.approx(v['drive_wheel_radius'])
    assert dd['linear.x.max_velocity'] == pytest.approx(v['max_linear_speed'])
    assert dd['linear.x.max_acceleration'] == pytest.approx(v['max_linear_accel'])
    assert dd['angular.z.max_velocity'] == pytest.approx(v['max_angular_speed'])
    assert dd['base_frame_id'] == 'base_footprint'
