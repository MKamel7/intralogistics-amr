"""Nobody writes quaternion to yaw again.

Extracting the duplication is a one-off; keeping it extracted is not. The
duplication this package removed was twelve copies across ten files, and none
of them arrived deliberately: each was written by someone who needed a heading
and did the obvious three lines rather than going looking for a helper. That
will keep happening, so this fails the build the next time it does.

It greps, and grep never proves coverage. A determined reimplementation with
different spacing or different variable names walks straight past it. The claim
here is narrower and still worth making: the two forms that were actually in
this repository cannot come back unnoticed.
"""

import re
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]
SEARCH = [ROOT / "src", ROOT / "tools"]
OWNER = "amr_common"

#: The two forms that were in the tree, with whitespace and line breaks free.
FULL_FORM = re.compile(r"atan2\s*\(\s*2\.?0?\s*\*\s*\(\s*q\.w\s*\*\s*q\.z", re.S)
SHORTCUT_FORM = re.compile(r"2\.?0?\s*\*\s*(?:math\.|np\.)?atan2\s*\(\s*q\.z\s*,\s*q\.w", re.S)


#: Build output, not source. A stray `src/amr_navigation/install/` exists on
#: this machine, full of symlinks into a workspace that has since been rebuilt,
#: so walking into it raises FileNotFoundError on a dangling link rather than
#: finding anything worth checking. Both are gitignored, so neither is source.
SKIP_DIRS = {"install", "build", "log", "__pycache__", OWNER}


def _python_files():
    for root in SEARCH:
        for path in root.rglob("*.py"):
            if SKIP_DIRS & set(path.parts):
                continue
            if not path.is_file():  # a dangling symlink is not a source file
                continue
            yield path


@pytest.mark.parametrize("pattern,name", [(FULL_FORM, "atan2(2(wz + xy), ...)"),
                                          (SHORTCUT_FORM, "2 * atan2(z, w)")])
def test_yaw_is_computed_in_one_place(pattern, name):
    offenders = [str(p.relative_to(ROOT)) for p in _python_files()
                 if pattern.search(p.read_text(encoding="utf-8"))]

    assert not offenders, (
        f"{name} is written again in {offenders}. Import "
        f"amr_common.pose.yaw_from_quaternion instead: the shortcut form is "
        f"wrong whenever roll and pitch are both non-zero, and the two forms "
        f"disagreeing by 5 degrees on a tilted vehicle is how this started.")


def test_the_gate_can_see_the_thing_it_is_looking_for():
    """Falsification, in the file rather than by hand.

    A grep gate that matches nothing looks identical to a clean tree, which is
    this project's own "a check that does not run is worse than no check".
    """
    assert FULL_FORM.search("yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0)")
    assert SHORTCUT_FORM.search("heading = 2.0 * math.atan2(q.z, q.w)")
    assert not FULL_FORM.search("yaw = yaw_from_quaternion(q)")
    assert not SHORTCUT_FORM.search("yaw = yaw_from_quaternion(q)")
