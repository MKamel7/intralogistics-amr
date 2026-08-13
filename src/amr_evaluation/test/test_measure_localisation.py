"""Guards on the localisation probe.

Each corresponds to a mistake already made by a probe in this project, which is
why they are worth asserting on a tool that needs a simulator to run.
"""

from pathlib import Path

PROBE = Path(__file__).resolve().parents[3] / 'tools' / 'measure_localisation.py'


def text():
    return PROBE.read_text()


def test_ground_truth_is_read_with_the_right_message_type():
    t = text()
    assert 'TFMessage' in t
    assert 'create_subscription(PoseArray' not in t
    assert 'import PoseArray' not in t


def test_the_vehicle_is_selected_by_frame_name():
    assert 'child_frame_id ==' in text(), (
        'index selection silently measures a pedestrian when the publication '
        'order changes')


def test_the_spawn_pose_is_read_not_typed():
    """A constant copied by hand into a diagnostic has produced a wrong answer
    in this project more than once."""
    t = text()
    assert 'stations_file' in t, 'the spawn must come from the stations file'
    assert 'yaml.safe_load' in t


def test_the_frame_assumption_is_checked_not_trusted():
    """At startup belief and truth agree by construction, so a large first
    sample means the frames are not related the way the tool assumes and every
    figure would be an offset rather than an error."""
    t = text()
    assert 'STARTUP_AGREEMENT' in t
    assert 'FRAME ASSUMPTION LOOKS WRONG' in t


def test_a_broken_tf_tree_is_distinguished_from_a_badly_localised_vehicle():
    """map and base_link ended up in two unconnected trees mid run. That is a
    different fault and must not be reported as zero error or as drift."""
    t = text()
    assert 'tf_failures' in t
    assert 'broken TF tree' in t


def test_the_parked_error_is_reported_separately():
    """Error at the moment of declared arrival is the figure that decides
    whether a pallet lands in the bay or beside it."""
    assert 'parked_errors' in text()


def test_it_reports_exactly_once():
    assert 'self.reported' in text()


def test_it_never_writes_anything():
    t = text()
    for bad in ('.write_text(', 'yaml.dump', "open(", 'w+'):
        if bad == 'open(':
            # reading the stations file is the one permitted open
            assert t.count('open(') == 1, 'only the stations file may be opened'
            continue
        assert bad not in t, f'the probe must not write ({bad})'
