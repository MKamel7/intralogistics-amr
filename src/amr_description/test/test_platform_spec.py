#!/usr/bin/env python3
"""Provenance gate for the platform specifications.

The rule this enforces: no physical constant enters the robot description
without a recorded source. A reviewer must never have to guess which numbers
came from a data sheet and which were made up.

These tests fail the build, not just warn, because a provenance rule nobody
enforces decays into a comment. They need no ROS and run in milliseconds.
"""

import math
from pathlib import Path

import pytest
import yaml

SPEC_DIR = Path(__file__).resolve().parents[1] / 'config' / 'platforms'
VALID_KINDS = {
    'datasheet',   # published by the manufacturer, archived
    'derived',     # computed from a datasheet figure
    'estimated',   # not published; a stated engineering judgement
    'tuned',       # chosen to make something behave, and admitted as such
    # MEASURED ON THE RUNNING SYSTEM. Added when self_filter_margin was
    # corrected from a derived 28.7 mm to a measured 40 mm: the derivation from
    # housing geometry was sound and incomplete, and the vehicle was observed
    # returning hits on its own structure 11 mm outside the derived margin.
    # This is the strongest kind available short of a datasheet, and the
    # taxonomy simply did not have a name for it.
    'measured',
}


def spec_files():
    files = sorted(SPEC_DIR.glob('*.yaml'))
    assert files, f'no platform specs found under {SPEC_DIR}'
    return files


@pytest.fixture(params=spec_files(), ids=lambda p: p.stem)
def spec(request):
    return yaml.safe_load(request.param.read_text())


@pytest.fixture
def platform_name(request):
    """The stem of whichever spec the `spec` fixture is currently serving."""
    return request.node.callspec.params['spec'].stem


def test_has_required_sections(spec):
    for section in ('platform', 'values', 'provenance', 'validation_targets'):
        assert section in spec, f'missing top-level section {section!r}'


def test_every_value_has_provenance(spec):
    missing = sorted(set(spec['values']) - set(spec['provenance']))
    assert not missing, (
        f'{len(missing)} value(s) with no recorded source: {missing}. '
        'Add an entry under provenance, or delete the value.')


def test_no_orphan_provenance(spec):
    """A source for a value that no longer exists means the spec drifted."""
    orphans = sorted(set(spec['provenance']) - set(spec['values']))
    assert not orphans, (
        f'provenance recorded for value(s) that do not exist: {orphans}')


def test_provenance_entries_are_well_formed(spec):
    for key, entry in spec['provenance'].items():
        assert isinstance(entry, dict), f'{key}: provenance must be a mapping'
        assert entry.get('kind') in VALID_KINDS, (
            f'{key}: kind {entry.get("kind")!r} is not one of {sorted(VALID_KINDS)}')
        src = entry.get('src', '')
        assert isinstance(src, str) and len(src.strip()) >= 15, (
            f'{key}: src must actually say where the number came from, got {src!r}')


# Words that indicate a source line gives a reason rather than just a label.
#
# This is a HEURISTIC and it is worth being honest about what it can do. It
# catches a bare label like "engineering estimate" with no reasoning attached.
# It cannot tell whether the reasoning given is correct, and it has already
# produced one false positive on a source line that explained itself at length
# but happened to use none of these words. When that happens the right response
# is to widen this list, not to reword prose to satisfy a keyword check.
#
# It also cannot catch the failure that actually matters most: a source that
# cites a document nobody read. Two of those have occurred in this project and
# both were found by reading the document, not by this test.
#
# WIDENED for a second false positive, mp400_class camera_update_rate. That
# source line names the constraint, says it is a CPU budget rather than a
# sensor limit, and quotes the control-loop rate it was measured against, which
# is more reasoning than most entries here carry. It simply happened to use
# none of these words. 'decision' and 'limit' are the forms it does use, and
# both attribute a value to something outside the number itself, which is what
# this list is for.
_REASONING_MARKERS = (
    'not published', 'not yet measured', 'because', 'chosen', 'taken',
    'typical', 'matches', 'halved', 'midpoint', 'proportionate', 'estimate of',
    'therefore', 'so that', 'so a', 'so the', '=',
    'decision', 'limit',
)


def test_non_datasheet_values_explain_themselves(spec):
    """derived, estimated and tuned all owe the reader a reason, not a label."""
    unexplained = []
    for key, entry in sorted(spec['provenance'].items()):
        if entry['kind'] == 'datasheet':
            continue
        src = entry['src'].lower()
        if not any(marker in src for marker in _REASONING_MARKERS):
            unexplained.append(f'{key} ({entry["kind"]}): {entry["src"]!r}')
    assert not unexplained, (
        'non-datasheet values must say WHY they hold their value, not just '
        'that they were estimated:\n  ' + '\n  '.join(unexplained))


def test_values_are_numeric_and_finite(spec):
    for key, value in spec['values'].items():
        assert isinstance(value, (int, float)) and not isinstance(value, bool), (
            f'{key}: platform values must be plain numbers, got {value!r}')
        assert math.isfinite(value), f'{key}: value is not finite'


def test_geometry_is_self_consistent(spec):
    """Cheap physical sanity checks that catch a fat-fingered edit."""
    v = spec['values']

    assert v['wheel_separation'] < v['chassis_width'], (
        'drive wheels sit outside the chassis width')
    assert 2 * v['drive_wheel_radius'] <= v['chassis_height'], (
        'drive wheel is taller than the chassis')
    assert v['caster_wheel_radius'] < v['drive_wheel_radius'], (
        'casters larger than drive wheels is not this class of machine')
    assert 2 * v['caster_inset_x'] < v['chassis_length'], (
        'casters sit outside the chassis length')
    assert 2 * v['caster_inset_y'] < v['chassis_width'], (
        'casters sit outside the chassis width')
    # The optical centre must sit AT OR PROUD OF the envelope corner, never
    # inboard of it. This assertion used to require the opposite, and that is
    # exactly what produced a contiguous 11.6 degree blind sector in the merged
    # scan: a scanner recessed inboard cannot see tangentially along the
    # vehicle's own sides, because the ray re-enters the chassis before
    # clearing it. See V-06 in docs/validation.md.
    assert 2 * v['scanner_mount_x'] >= v['chassis_length'], (
        f"scanner optical centre at x={v['scanner_mount_x']} is inboard of the "
        f"{v['chassis_length'] / 2} m envelope corner; the vehicle's own side "
        f"will block its tangential view and open a blind sector")
    assert 2 * v['scanner_mount_y'] >= v['chassis_width'], (
        f"scanner optical centre at y={v['scanner_mount_y']} is inboard of the "
        f"{v['chassis_width'] / 2} m envelope corner")
    # ...but only just. A sensor sticking far out of the envelope is a different
    # machine from the one on the data sheet.
    proud_x = v['scanner_mount_x'] - v['chassis_length'] / 2
    proud_y = v['scanner_mount_y'] - v['chassis_width'] / 2
    assert proud_x <= 0.02 and proud_y <= 0.02, (
        f'scanner optics stand {proud_x * 1000:.0f} x {proud_y * 1000:.0f} mm proud '
        f'of the published envelope, which is no longer that platform')
    assert v['scanner_housing_inset'] > 0.5 * v['scanner_body_x'], (
        'housing inset is too small to pull the sensor body back inside the '
        'corner recess')
    assert v['scanner_mount_height'] < v['chassis_height'], (
        'scan plane is above the deck, which would not see a person at floor level')
    assert v['scanner_protective_range'] < v['scanner_warning_range'], (
        'protective field must sit inside the warning field')
    assert v['scanner_warning_range'] <= v['scanner_measuring_range'], (
        'warning field cannot exceed the measuring range')


# Platforms whose self filter is still the plain bounding box, and why.
#
# Shaping the filter needs the pods measured on that platform. The MiR250
# figure is not a pod derivation at all: 0.060 m came from returns at a fixed
# 0.440 m from centre, 12 of 12 protective stops in one run, minimum equal to
# median equal to maximum and absent from both costmaps. Its own pod geometry
# only reaches 0.4287 m, so 11 mm of that measurement is something else on the
# vehicle and nobody has identified it. Shrinking that margin on the strength
# of the MP-400 result would be exactly the mistake V-45 punished, where a rule
# derived from one platform dropped the other from 3 of 3 cycles to 2 of 9.
#
# So V-39 stays OPEN on the MiR250, deliberately, and this list is the record.
# An entry is asserted to be stale the moment that platform declares pods.
UNSHAPED_SELF_FILTER = {
    'mir250_class':
        'V-39 open: the 40 mm of self return measured on this platform is not '
        'explained by its pod geometry, which reaches 28.7 mm, and the '
        'remaining 11 mm has never been identified. Not shaped until it is.',
}


def test_pod_geometry_matches_the_primitives(spec, platform_name):
    """The pod boxes are derived, so they must not drift from what they derive from.

    self_pod_x, self_pod_y and self_pod_half are carried as spec values because
    the launch file and the scan merger both need them and neither is a good
    place for trigonometry. That convenience is only safe while they still
    agree with scanner_mount_x, scanner_mount_y, scanner_housing_inset and the
    body dimensions. Move the mount without moving these and the filter would
    reject a region the vehicle no longer occupies while missing one it does.
    """
    v = spec['values']
    if platform_name in UNSHAPED_SELF_FILTER:
        assert 'self_pod_x' not in v, (
            f'{platform_name} declares pods now, so its UNSHAPED_SELF_FILTER '
            f'entry is stale and must be deleted along with this branch')
        pytest.skip(UNSHAPED_SELF_FILTER[platform_name])
    c = math.cos(math.pi / 4.0)
    expected = {
        'self_pod_x': v['scanner_mount_x'] - v['scanner_housing_inset'] * c,
        'self_pod_y': v['scanner_mount_y'] - v['scanner_housing_inset'] * c,
        'self_pod_half': (v['scanner_body_x'] / 2.0) * c + (v['scanner_body_y'] / 2.0) * c,
    }
    for key, want in expected.items():
        assert abs(v[key] - want) < 1e-6, (
            f'{key} is {v[key]:.6f} but the primitives give {want:.6f}; the '
            f'pod geometry has drifted from the mount it is derived from')


def test_the_self_filter_rejects_the_whole_vehicle(spec, platform_name):
    """Every part of the vehicle must fall inside the filter, pods included.

    The optics sit 5 mm proud of the envelope corner and the housing is a box
    rotated 45 degrees, so the pods reach 28.7 mm past the chassis. When
    nothing covered that, those returns survived the self filter, landed inside
    the protective field, and the vehicle held a permanent protective stop
    against its own corners without moving once.

    What changed in V-49 is HOW they are covered. The margin used to be wide
    enough to swallow them, which blanked 28.7 mm along every side including
    the ones with no pod on them. Now the pods are modelled and the margin only
    has to clear the chassis. This test still demands full coverage; it just no
    longer accepts a margin as the way to get it.
    """
    v = spec['values']
    if platform_name in UNSHAPED_SELF_FILTER:
        assert 'self_pod_x' not in v, (
            f'{platform_name} declares pods now, so its UNSHAPED_SELF_FILTER '
            f'entry is stale and must be deleted along with this branch')
        pytest.skip(UNSHAPED_SELF_FILTER[platform_name])
    c = math.cos(math.pi / 4.0)
    half = v['self_pod_half']

    # The pod corner is the extreme point of the whole vehicle, and it is the
    # pod box rather than the body box that has to contain it. So what is
    # checked here is that the pod box was not drawn smaller than the housing
    # it stands for.
    assert half >= (v['scanner_body_x'] / 2.0) * c, (
        'pod half extent is smaller than the rotated housing, so the corner of '
        'the scanner itself falls outside the self filter')

    # And the optics, which sit further out than the housing does.
    assert v['self_pod_y'] + half >= v['scanner_mount_y'], (
        'the pod box does not reach the optical centre, so the scanner would '
        'see its own aperture')

    # And the margin must still cover what the vehicle was MEASURED to see of
    # itself beyond the chassis, which is a different quantity from the pods.
    assert v['self_filter_margin'] >= 0.0030, (
        f'self filter margin {v["self_filter_margin"] * 1000:.1f} mm is below '
        f'the 3.0 mm of self observation measured on this platform')


def test_the_self_filter_is_not_wider_than_the_vehicle_where_there_is_no_pod(
        spec, platform_name):
    """The other half of V-39, and the reason the filter is shaped at all.

    A blind zone along the flank costs lateral coverage in the forward
    protective fields, which are only scanner_protective_supplement wider than
    the chassis to begin with. The pods are corner-local: they span x from
    0.191 to 0.324, so along the middle of each side the vehicle is nothing but
    chassis and the filter must not pretend otherwise.

    Two earlier attempts to buy that coverage by enlarging the FIELDS were both
    measured to be worse than the defect (V-42, V-45). Shrinking the filter
    where the vehicle is genuinely smaller costs nothing.
    """
    v = spec['values']
    if platform_name in UNSHAPED_SELF_FILTER:
        assert 'self_pod_x' not in v, (
            f'{platform_name} declares pods now, so its UNSHAPED_SELF_FILTER '
            f'entry is stale and must be deleted along with this branch')
        pytest.skip(UNSHAPED_SELF_FILTER[platform_name])
    band = (v['chassis_width'] / 2.0 + v['scanner_protective_supplement']) - (
        v['chassis_width'] / 2.0 + v['self_filter_margin'])
    assert band >= 0.050, (
        f'the self filter leaves forward protective fields {band * 1000:.1f} mm '
        f'of lateral coverage along the flank, below the 50 mm a person needs '
        f'to produce the two returns min_points requires. This is V-39')

    # The pods may not creep along the side and undo that.
    assert v['self_pod_x'] - v['self_pod_half'] > 0.15, (
        'the pod boxes now reach the middle of the side, where they blank the '
        'flank coverage the shaped filter exists to protect')


def test_scanner_pair_covers_360_degrees(spec):
    """The reason two scanners are specified at all.

    Two apertures at diagonally opposite corners must sum past a full turn, or
    the robot has a blind sector and the safety concept is void.
    """
    angle = spec['values']['scanner_scan_angle']
    assert 2 * angle >= 360.0, (
        f'two {angle} deg scanners cover {2 * angle} deg, leaving a blind sector')


def test_scanner_sample_count_matches_angular_resolution(spec):
    v = spec['values']
    expected = math.ceil(v['scanner_scan_angle'] / v['scanner_angular_resolution'])
    assert v['scanner_samples'] == expected, (
        f'scanner_samples {v["scanner_samples"]} disagrees with '
        f'{v["scanner_scan_angle"]} deg / {v["scanner_angular_resolution"]} deg '
        f'= {expected}')


def test_scanner_update_rate_matches_response_time(spec):
    v = spec['values']
    expected = 1.0 / v['scanner_response_time']
    assert abs(v['scanner_update_rate'] - expected) < 0.1, (
        f'scanner_update_rate {v["scanner_update_rate"]} Hz is not the '
        f'{expected:.1f} Hz implied by a {v["scanner_response_time"]} s response time')


def test_robot_fits_the_corridor_it_claims(spec):
    """The published corridor widths are the headline validation targets.

    A robot wider than the corridor it is supposed to drive down means the spec
    was transcribed wrong, and everything downstream would be measured against a
    target that cannot be met.
    """
    v, t = spec['values'], spec['validation_targets']
    assert v['chassis_width'] < t['corridor_width_dynamic'], (
        f'chassis {v["chassis_width"]} m does not fit the '
        f'{t["corridor_width_dynamic"]} m dynamic-footprint corridor target')
    diagonal = math.hypot(v['chassis_length'], v['chassis_width'])
    assert diagonal > t['corridor_width_90_turn'], (
        'body diagonal is smaller than the 90 degree corridor target, which '
        'would make that target trivially satisfiable and not worth asserting')


def test_camera_pair_spans_the_published_combined_field_of_view(spec):
    """Two sheets, one number, and they have to agree.

    The platform sheet publishes 114 degrees horizontal for the pair; the Intel
    sheet gives 87 degrees for one D435. The toe-out is derived from those two,
    so this asserts the derivation still holds. It also documents the resolution
    of an ambiguity that was live for several commits: 114 is COMBINED, not per
    camera, and modelling each camera at 114 overstated the sensor by 27
    degrees.
    """
    v, t = spec['values'], spec['validation_targets']
    combined = v['camera_hfov'] + 2.0 * v['camera_splay']
    assert combined == pytest.approx(t['camera_combined_hfov'], abs=0.5), (
        f"two {v['camera_hfov']} deg cameras toed out {v['camera_splay']} deg span "
        f"{combined} deg, but the platform sheet publishes "
        f"{t['camera_combined_hfov']} deg for the pair")
    assert v['camera_hfov'] < t['camera_combined_hfov'], (
        'a single camera cannot be as wide as the pair; that was the original error')


def test_camera_clip_planes_are_the_sensor_limits(spec):
    """Min-Z and maximum range are sensor properties, not framing choices."""
    v = spec['values']
    assert v['camera_min_depth'] < v['camera_min_ground_view'], (
        'the sensor minimum depth should be nearer than the mounting-derived '
        'minimum ground view; these are different quantities and were conflated')
    assert v['camera_max_range'] > v['camera_fov_distance']


def test_validation_targets_are_not_silently_dropped(spec):
    """These are the claims the project promises to measure itself against."""
    required = {
        'corridor_width_dynamic', 'corridor_width_90_turn',
        'min_object_at_1m', 'distance_to_max_speed',
        'runtime_loaded_h', 'runtime_unloaded_h',
    }
    missing = sorted(required - set(spec['validation_targets']))
    assert not missing, f'validation targets removed from the spec: {missing}'
