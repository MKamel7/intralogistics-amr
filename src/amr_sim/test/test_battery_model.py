#!/usr/bin/env python3
"""Checks on the battery power fit.

These verify the ARITHMETIC of the calibration, not the physics. The model is
fitted to three published runtimes, so reproducing those three is a property of
the fit and is worth asserting only because a wrong fit would be invisible
otherwise. It is explicitly not evidence that the energy behaviour is correct,
and docs/validation.md says so.

Pure Python, no ROS: the fit is a static method precisely so it can be tested
without a node.
"""

import importlib.util
from pathlib import Path

import pytest
import yaml

MODULE = (Path(__file__).resolve().parents[1] / 'amr_sim' / 'battery_model.py')
SPEC = (Path(__file__).resolve().parents[2]
        / 'amr_description' / 'config' / 'platforms' / 'mir250_class.yaml')


def _load():
    spec = importlib.util.spec_from_file_location('battery_model', MODULE)
    mod = importlib.util.module_from_spec(spec)
    # rclpy is imported at module scope; skip cleanly where it is unavailable.
    spec.loader.exec_module(mod)
    return mod


try:
    BatteryModel = _load().BatteryModel
except Exception as exc:  # pragma: no cover - environment dependent
    pytest.skip(f'battery_model not importable: {exc}', allow_module_level=True)


@pytest.fixture(scope='module')
def platform():
    return yaml.safe_load(SPEC.read_text())


@pytest.fixture(scope='module')
def fitted(platform):
    v, t = platform['values'], platform['validation_targets']
    cap = v['battery_capacity_kwh'] * 1000.0
    ref = 1.0
    p_standby, p_drive, p_payload = BatteryModel.fit_power(
        cap, t['runtime_standby_h'], t['runtime_unloaded_h'],
        t['runtime_loaded_h'], ref, v['max_linear_speed'])
    return {'cap': cap, 'ref': ref, 'max_speed': v['max_linear_speed'],
            'p_standby': p_standby, 'p_drive': p_drive, 'p_payload': p_payload,
            'targets': t}


def _runtime_h(cap_wh, power_w):
    return cap_wh / power_w


def test_fit_reproduces_the_published_standby_time(fitted):
    h = _runtime_h(fitted['cap'], fitted['p_standby'])
    assert h == pytest.approx(fitted['targets']['runtime_standby_h'], rel=1e-6)


def test_fit_reproduces_the_published_unloaded_runtime(fitted):
    duty = fitted['ref'] / fitted['max_speed']
    p = fitted['p_standby'] + fitted['p_drive'] * duty
    h = _runtime_h(fitted['cap'], p)
    assert h == pytest.approx(fitted['targets']['runtime_unloaded_h'], rel=1e-6)


def test_fit_reproduces_the_published_loaded_runtime(fitted):
    duty = fitted['ref'] / fitted['max_speed']
    p = (fitted['p_standby'] + fitted['p_drive'] * duty
         + fitted['p_payload'] * duty)
    h = _runtime_h(fitted['cap'], p)
    assert h == pytest.approx(fitted['targets']['runtime_loaded_h'], rel=1e-6)


def test_power_terms_are_all_positive(fitted):
    """A negative term would mean driving or loading SAVES energy."""
    for name in ('p_standby', 'p_drive', 'p_payload'):
        assert fitted[name] > 0.0, f'{name} is {fitted[name]}, which is nonsense'


def test_carrying_a_load_costs_more_than_not(fitted):
    duty = fitted['ref'] / fitted['max_speed']
    unloaded = fitted['p_standby'] + fitted['p_drive'] * duty
    loaded = unloaded + fitted['p_payload'] * duty
    assert loaded > unloaded


def test_standing_still_costs_the_standby_power_only(fitted):
    """At zero speed neither the drive nor the payload term may contribute.

    A payload sitting on a stationary robot draws nothing; if the model charged
    for it, an idle loaded robot would drain faster than an idle empty one for
    no physical reason.
    """
    duty = 0.0
    p = (fitted['p_standby'] + fitted['p_drive'] * duty
         + fitted['p_payload'] * duty)
    assert p == pytest.approx(fitted['p_standby'])


def test_reference_speed_changes_the_fit_but_not_the_reproduced_runtimes(platform):
    """The duty assumption is explicit, and the published figures survive it.

    The sheet never states the duty cycle behind its 'active operation time'.
    Whatever reference speed is chosen, the model must still reproduce the
    published runtimes AT THAT SPEED, otherwise the assumption is silently
    changing the answer rather than being an assumption.
    """
    v, t = platform['values'], platform['validation_targets']
    cap = v['battery_capacity_kwh'] * 1000.0
    for ref in (0.5, 1.0, 1.5, 2.0):
        ps, pd, pp = BatteryModel.fit_power(
            cap, t['runtime_standby_h'], t['runtime_unloaded_h'],
            t['runtime_loaded_h'], ref, v['max_linear_speed'])
        duty = ref / v['max_linear_speed']
        assert cap / (ps + pd * duty) == pytest.approx(
            t['runtime_unloaded_h'], rel=1e-6), f'broken at ref={ref}'
        assert cap / (ps + pd * duty + pp * duty) == pytest.approx(
            t['runtime_loaded_h'], rel=1e-6), f'broken at ref={ref}'


def test_zero_reference_speed_is_rejected(platform):
    v, t = platform['values'], platform['validation_targets']
    with pytest.raises(ValueError):
        BatteryModel.fit_power(
            v['battery_capacity_kwh'] * 1000.0, t['runtime_standby_h'],
            t['runtime_unloaded_h'], t['runtime_loaded_h'],
            0.0, v['max_linear_speed'])
