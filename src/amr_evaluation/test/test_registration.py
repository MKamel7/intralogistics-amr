"""Every test file is actually run by the build.

WHY THIS EXISTS

Eleven test files across three packages were never registered in their
package's CMakeLists.txt, so `colcon test` never ran them: 289 cases against
the 337 a direct `pytest src` collects. They all passed, so nothing ever drew
attention to it.

The container was fine, because it runs `pytest src` rather than `colcon test`.
What was not fine is that the build's own verdict on the workspace was missing
a seventh of the suite, including every probe that produces a safety number.

That is the same shape as the two other silent gaps this project has found: a
strict xfail nobody reads (V-49), and a package whose tests exited 5 because
pytest collected nothing (`amr_vda5050`, which was `ament_python` and reported
NO TESTS RAN as a package failure for as long as it existed).

The pattern is worth naming: **a check that does not run is worse than no
check, because it is counted.** The test suite figure quoted in the README and
the handover is the number people trust when they change a protective field.

This walks the source tree rather than any list, so a new test file is covered
the day it is written.
"""

import re
from pathlib import Path

SRC = Path(__file__).resolve().parents[3] / 'src'

# ament_python packages register nothing; their tests are collected by the
# build tool itself. There are none at present: amr_vda5050 was converted to
# ament_cmake precisely because that collection did not work.
CMAKE_ONLY = True


def registered(pkg):
    cml = pkg / 'CMakeLists.txt'
    if not cml.exists():
        return None
    return set(re.findall(r'ament_add_pytest_test\(\s*([A-Za-z0-9_]+)', cml.read_text()))


def present(pkg):
    return {p.stem for p in (pkg / 'test').glob('test_*.py')}


def packages():
    return sorted(p for p in SRC.iterdir() if (p / 'package.xml').exists())


def test_every_test_file_is_registered():
    missing = {}
    for pkg in packages():
        if not (pkg / 'test').is_dir():
            continue
        reg = registered(pkg)
        assert reg is not None, (
            f'{pkg.name} has a test/ directory and no CMakeLists.txt, so '
            f'nothing states how its tests are meant to run')
        gap = present(pkg) - reg
        if gap:
            missing[pkg.name] = sorted(gap)
    assert not missing, (
        'these test files exist but are not registered with '
        'ament_add_pytest_test, so `colcon test` and the container never run '
        'them:\n  ' + '\n  '.join(
            f'{k}: {", ".join(v)}' for k, v in sorted(missing.items())))


def test_no_registration_points_at_a_missing_file():
    """The other direction. A stale entry fails the build loudly, but only
    once someone builds; naming it here says what happened."""
    dangling = {}
    for pkg in packages():
        reg = registered(pkg)
        if not reg:
            continue
        gap = reg - present(pkg)
        if gap:
            dangling[pkg.name] = sorted(gap)
    assert not dangling, (
        'these registrations name test files that do not exist:\n  ' +
        '\n  '.join(f'{k}: {", ".join(v)}' for k, v in sorted(dangling.items())))


def test_every_package_is_ament_cmake():
    """Because the one that was not had its tests silently uncollected.

    `amr_vda5050` was `ament_python`. `colcon test` ran pytest in a directory
    where it found nothing, exited 5, and reported the package as FAILED while
    its nineteen protocol tests passed under a direct run. Keeping the build
    type uniform is what makes the registration check above sufficient.
    """
    odd = []
    for pkg in packages():
        text = (pkg / 'package.xml').read_text()
        m = re.search(r'<build_type>([^<]+)</build_type>', text)
        if m and m.group(1).strip() != 'ament_cmake':
            odd.append(f'{pkg.name} is {m.group(1).strip()}')
    assert not odd, (
        'these packages are not ament_cmake, so test_every_test_file_is_'
        'registered does not cover them:\n  ' + '\n  '.join(odd))
