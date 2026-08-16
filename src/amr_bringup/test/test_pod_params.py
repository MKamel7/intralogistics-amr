"""The scan merger's pod parameters, for every platform in the tree.

WHY THIS EXISTS

The launch read `v['self_pod_x']` unconditionally. The MP-400 declares pods
and the MiR250 does not, so bringing up the MiR250 would have raised KeyError
before the simulator started. Nothing caught it: the launch is only exercised
against the platform that has them, and the platform spec tests never touch
the launch.

That is the third defect in this project of the same shape, after V-49 and
V-50. A check that covers one case and is assumed to cover the rest is not
covering the rest.

This walks the platform directory rather than naming platforms, so a new spec
is covered the day it is written.
"""

import importlib.util
from pathlib import Path

import pytest
import yaml

REPO = Path(__file__).resolve().parents[3]
LAUNCH = REPO / 'src' / 'amr_bringup' / 'launch' / 'robot.launch.py'
SPECS = REPO / 'src' / 'amr_description' / 'config' / 'platforms'


def load():
    spec = importlib.util.spec_from_file_location('robot_launch', LAUNCH)
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


def platforms():
    return sorted(SPECS.glob('*.yaml'))


@pytest.mark.parametrize('path', platforms(), ids=lambda p: p.stem)
def test_every_platform_produces_pod_parameters(path):
    v = yaml.safe_load(path.read_text())['values']
    pods = load().pod_params(v)

    assert set(pods) == {'self_pod_x', 'self_pod_y', 'self_pod_half'}
    n = len(pods['self_pod_x'])
    assert all(len(pods[k]) == n for k in pods), (
        f'{path.stem} produces pod lists of different lengths, which the scan '
        f'merger rejects at construction')

    if 'self_pod_x' not in v:
        assert n == 0, (
            f'{path.stem} declares no pod geometry but the launch invented '
            f'{n} of them')
        return

    # Declared pods must be the diagonal pair, and must be real boxes. A pod
    # with a non-positive half extent filters nothing while the chassis margin
    # has already been reduced to suit it, which is a self return reaching the
    # protective field.
    assert n == 2, f'{path.stem} produces {n} pods; the pair is diagonal'
    assert pods['self_pod_x'][0] == -pods['self_pod_x'][1]
    assert pods['self_pod_y'][0] == -pods['self_pod_y'][1]
    assert all(h > 0.0 for h in pods['self_pod_half'])


@pytest.mark.parametrize('path', platforms(), ids=lambda p: p.stem)
def test_a_platform_with_pods_declares_all_three(path):
    """Half a pod is worse than none.

    self_pod_x on its own would raise KeyError deep inside the launch, at
    which point the simulator is already up and the failure looks like a
    bringup race rather than a missing constant.
    """
    v = yaml.safe_load(path.read_text())['values']
    keys = [k for k in ('self_pod_x', 'self_pod_y', 'self_pod_half') if k in v]
    assert len(keys) in (0, 3), (
        f'{path.stem} declares {keys} and not the rest of the pod geometry')
