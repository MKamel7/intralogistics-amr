"""The fleet launch is a specification until the bringup can be namespaced.

This project's convention is that a claim carries the measurement or the test
that backs it. The fleet launch backs nothing yet, so the gap is asserted here
rather than left for someone to discover by running it.
"""

from pathlib import Path

import pytest

PKG = Path(__file__).resolve().parents[1]
BRINGUP = PKG.parents[0] / 'amr_bringup' / 'launch' / 'robot.launch.py'


def test_the_fleet_launch_says_it_does_not_run():
    """Presenting it as a working entry point would be an unsupported claim."""
    t = (PKG / 'launch' / 'fleet.launch.py').read_text()
    assert 'THIS DOES NOT RUN YET' in t


@pytest.mark.xfail(strict=True, reason=(
    'The bringup cannot be namespaced yet. Every node publishes fixed topic '
    'names and the description emits fixed frame names, so two vehicles fight '
    'rather than coexist: both drive /cmd_vel_raw, both write /map, and both '
    'publish odom -> base_link into one TF tree. Namespacing touches the '
    'description, Nav2, the collision monitor, the scan merger and the spawn '
    'path. Strict, so this XPASSes the moment the arguments exist and forces '
    'the fleet launch to stop calling itself a specification.'))
def test_the_bringup_accepts_the_arguments_the_fleet_launch_passes():
    t = BRINGUP.read_text()
    for arg in ('namespace', 'tf_prefix', 'start_simulator'):
        assert f"'{arg}'" in t, f'robot.launch.py does not declare {arg}'
