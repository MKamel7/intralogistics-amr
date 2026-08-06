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
VALID_KINDS = {'datasheet', 'derived', 'estimated', 'tuned'}


def spec_files():
    files = sorted(SPEC_DIR.glob('*.yaml'))
    assert files, f'no platform specs found under {SPEC_DIR}'
    return files


@pytest.fixture(params=spec_files(), ids=lambda p: p.stem)
def spec(request):
    return yaml.safe_load(request.param.read_text())


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
# Deliberately loose: the point is to catch 'estimated' with no explanation at
# all, not to police prose.
_REASONING_MARKERS = (
    'not published', 'because', 'chosen', 'taken', 'typical', 'matches',
    'halved', 'midpoint', 'proportionate', 'so that', 'so a', 'so the', '=',
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
    assert 2 * v['scanner_mount_x'] < v['chassis_length'], (
        'scanner mounted outside the chassis length')
    assert v['scanner_mount_height'] < v['chassis_height'], (
        'scan plane is above the deck, which would not see a person at floor level')
    assert v['scanner_protective_range'] < v['scanner_warning_range'], (
        'protective field must sit inside the warning field')
    assert v['scanner_warning_range'] <= v['scanner_measuring_range'], (
        'warning field cannot exceed the measuring range')


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


def test_validation_targets_are_not_silently_dropped(spec):
    """These are the claims the project promises to measure itself against."""
    required = {
        'corridor_width_dynamic', 'corridor_width_90_turn',
        'min_object_at_1m', 'distance_to_max_speed',
        'runtime_loaded_h', 'runtime_unloaded_h',
    }
    missing = sorted(required - set(spec['validation_targets']))
    assert not missing, f'validation targets removed from the spec: {missing}'
