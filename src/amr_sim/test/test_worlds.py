"""The worlds parse, and every model they include is actually present.

WHY THIS IS NOT SIMPLY "RUN gz sdf"

The handover claimed `gz sdf` validates both worlds. It does not and cannot
validate `warehouse.sdf`, and the tool says so plainly:

    Tried to use callback in sdf::findFile(), but the callback is empty.
    Did you call sdf::setFindCallback()?

`model://` resolution is installed by gz-sim at runtime. The standalone
validator has no such callback, so every `<include><uri>model://...` in a world
is unresolvable to it, and it reports 25 errors on a world that loads and runs
perfectly well. The generated test tracks pass only because they are
self contained and include nothing.

So the claim was checking a property one world could never have, and no test
backed it. What it was reaching for is the invariant below: a world that
references a model which is not installed comes up with a hole in it, and the
first sign is a robot driving through a shelf that was never spawned.
"""

import re
import shutil
import subprocess
from pathlib import Path

import pytest

PKG = Path(__file__).resolve().parents[1]
WORLDS = PKG / 'worlds'
MODELS = PKG / 'models'


def worlds():
    return sorted(WORLDS.glob('*.sdf'))


def included_models(path):
    """Every `model://name` the world references."""
    return sorted(set(re.findall(r'model://([A-Za-z0-9_\-]+)', path.read_text())))


def test_there_are_worlds_to_check():
    """An empty glob is a green tick that checked nothing. See V-50."""
    assert len(worlds()) >= 2, f'only {len(worlds())} worlds found in {WORLDS}'


@pytest.mark.parametrize('path', worlds(), ids=lambda p: p.stem)
def test_every_included_model_is_present(path):
    """The invariant the gz sdf claim was actually about.

    A world referencing a model that is not installed loads with that object
    missing, silently. Nothing in the logs says a shelf is absent; the vehicle
    simply drives through where it should have been, and the run looks normal.
    """
    missing = [m for m in included_models(path) if not (MODELS / m).is_dir()]
    assert not missing, (
        f'{path.name} includes models that are not in {MODELS}:\n  ' +
        '\n  '.join(missing))


@pytest.mark.parametrize('path', worlds(), ids=lambda p: p.stem)
def test_every_included_model_has_a_config(path):
    """`model://` resolves by finding model.config, not by directory name."""
    bad = [m for m in included_models(path)
           if (MODELS / m).is_dir() and not (MODELS / m / 'model.config').is_file()]
    assert not bad, (
        f'{path.name} includes models whose directory exists but carries no '
        f'model.config, so gz cannot resolve them:\n  ' + '\n  '.join(bad))


@pytest.mark.parametrize('path', worlds(), ids=lambda p: p.stem)
def test_the_world_is_well_formed_xml(path):
    """Cheap, and it holds for every world including the ones gz sdf cannot
    check. A truncated write is the failure this catches."""
    import xml.etree.ElementTree as ET
    root = ET.parse(path).getroot()
    assert root.tag == 'sdf', f'{path.name} root element is {root.tag!r}'
    assert root.find('world') is not None, f'{path.name} declares no world'


@pytest.mark.skipif(shutil.which('gz') is None, reason='gz not on PATH')
@pytest.mark.parametrize(
    'path', [p for p in worlds() if not included_models(p)],
    ids=lambda p: p.stem)
def test_self_contained_worlds_pass_the_sdf_validator(path):
    """Only the worlds that include nothing.

    Parametrised over the self contained ones rather than skipping inside the
    test, so the ones that cannot be validated this way are visibly absent from
    the report instead of appearing as passes.
    """
    r = subprocess.run(['gz', 'sdf', '-k', str(path)],
                       capture_output=True, text=True, timeout=60)
    assert r.returncode == 0, (
        f'{path.name} failed sdf validation:\n{r.stderr[:2000]}')
