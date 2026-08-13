"""Guards on the slip probe, all of which correspond to a mistake made once.

The probe itself needs a running simulator, so what is checked here is the set
of properties that were got wrong in writing it and would be silent if got
wrong again.
"""

from pathlib import Path

PROBE = Path(__file__).resolve().parents[3] / 'tools' / 'measure_slip.py'


def text():
    return PROBE.read_text()


def test_subscribes_with_the_right_message_type():
    """Ground truth is tf2_msgs/TFMessage. Subscribing with the wrong type is
    completely silent: the subscription is created, nothing ever matches, and
    the probe reports no motion, which is indistinguishable from a stationary
    vehicle. The same mistake cost this project thirty four protective stops
    of latency data before it was noticed.
    """
    t = text()
    assert 'TFMessage' in t, 'ground truth is tf2_msgs/TFMessage'
    # The word appears in the comment that records why it is wrong, which is
    # worth keeping, so check the import and the subscription rather than the
    # text. An over-strict test here failed on its own explanatory comment.
    assert 'import PoseArray' not in t, 'PoseArray must not be imported'
    assert 'create_subscription(PoseArray' not in t, (
        'PoseArray was the first guess and it is the wrong type; a subscription '
        'that never matches reports zero motion rather than an error')
    assert 'create_subscription(TFMessage' in t


def test_picks_the_vehicle_by_frame_name_not_by_index():
    """The ground truth stream carries the vehicle among the pedestrians.
    Taking transforms[0] works until the publication order changes, and then
    silently measures a pedestrian.
    """
    t = text()
    assert 'child_frame_id ==' in t, 'the vehicle must be selected by frame name'
    assert 'msg.transforms[0]' not in t, 'index selection breaks on reordering'


def test_refuses_to_divide_a_ratio_out_of_noise():
    """A stationary vehicle gives 0/0. Printing a ratio there would be a
    number with no meaning, and it would be believed."""
    t = text()
    assert 'self.odom_path < 0.10' in t, (
        'there must be a floor below which no ratio is reported')
    assert 'Not a verdict either way' in t, (
        'the no-motion case must say it is not a result')


def test_reports_exactly_once():
    """report() is reachable from the timer and from the finally block. Without
    a guard the summary prints twice, and the duplicate reads as a second,
    confirming measurement."""
    assert 'self.reported' in text(), 'report must be idempotent'


def test_never_writes_a_spec():
    """A tool that edited the platform spec from its own measurement would be
    one bad run away from changing the vehicle's model."""
    t = text()
    for bad in ('.write_text(', 'yaml.dump', 'open(', 'w+'):
        assert bad not in t, f'the probe must not write anything ({bad})'


def test_the_acceptance_band_is_tight_enough_to_fail():
    """The bands were 0.7 to 1.3 and that is why this tool lied.

    It measured a ratio of 0.744, which was a 34 percent odometry scale error
    corrupting every SLAM update, and printed "odometry is consistent with
    ground truth" because 0.744 falls inside 0.7 to 1.3. A tolerance that wide
    is not a tolerance, it is a guarantee of passing.
    """
    t = text()
    assert 'ratio < 0.7' not in t, (
        'the 0.7 lower band is what reported a 34 percent error as consistent')
    assert 'ratio > 1.3' not in t, 'the 1.3 upper band is equally permissive'
    assert 'err <= 0.02' in t, (
        'odometry feeding scan matching must agree to a few percent')


def test_it_names_the_constant_to_check_rather_than_the_symptom():
    """A scale error presents as a mapping problem. Four hypotheses in this
    project went to SLAM tuning before anyone compared the wheel radius."""
    t = text()
    assert 'wheel_radius' in t and 'wheel_separation' in t, (
        'the verdict must point at the controller constants')
    assert 'platform spec' in t
