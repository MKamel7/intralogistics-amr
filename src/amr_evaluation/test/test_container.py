"""The container definition, checked as text.

Building the image takes minutes and needs a network, so it is not done here.
What is checked is the set of properties that were reasoned about when writing
it, each of which would otherwise be silently lost in a later edit.
"""

from pathlib import Path

REPO = Path(__file__).resolve().parents[3]


def dockerfile():
    return (REPO / 'Dockerfile').read_text()


def compose():
    return (REPO / 'docker-compose.yml').read_text()


def test_the_base_matches_the_distribution_the_project_targets():
    assert 'FROM ros:jazzy' in dockerfile(), (
        'the project is Jazzy; a different base would build something that is '
        'not this project')


def test_gazebo_harmonic_is_installed_explicitly():
    """Harmonic is not in the ROS base image, and the pairing of Jazzy with
    Harmonic is the whole platform decision recorded in the ADRs."""
    t = dockerfile()
    assert 'gz-harmonic' in t
    assert 'packages.osrfoundation.org' in t


def test_dependencies_are_pinned_rather_than_resolved_at_build_time():
    """A rosdep run six months from now resolves against a different index, so
    the build stops being a record of anything. The list is explicit on
    purpose and the Dockerfile says why."""
    t = dockerfile()
    assert 'rosdep install' not in t
    assert 'reproducible' in t


def test_it_builds_with_symlink_install():
    """Half the tests assume a generator run updates the installed config,
    which is only true with --symlink-install."""
    assert '--symlink-install' in dockerfile()


def test_it_is_headless_by_default():
    """Without this Gazebo tries to open a render window and fails in a way
    that reads as a simulation fault rather than a missing display."""
    t = dockerfile()
    assert 'QT_QPA_PLATFORM=offscreen' in t


def test_the_default_command_is_the_test_suite():
    """The tests are the part a reader can check in ninety seconds without a
    simulator, and they are what carries the measurements."""
    assert 'CMD ["pytest"]' in dockerfile()


def test_compose_raises_shared_memory():
    """Gazebo and the ROS middleware share memory. The 64 MB default produces
    transport failures that look like sensor faults."""
    t = compose()
    assert 'shm_size' in t
    assert 'look like sensor faults' in t


def test_compose_does_not_pretend_to_offer_a_gui():
    """Rendering in a container needs a display socket and GPU passthrough
    that differ per machine. An image that only works where it was written is
    worse than one honest about its scope."""
    t = compose()
    assert 'NO GUI SERVICE HERE ON PURPOSE' in t
    assert 'rviz' not in t.lower().split('no gui service here on purpose')[0]


def test_the_entrypoint_sources_the_workspace():
    t = (REPO / 'docker-entrypoint.sh').read_text()
    assert 'install/setup.bash' in t
    assert 'set -euo pipefail' in t


def test_build_artefacts_are_not_copied_into_the_image():
    """Copying a host build into the image produces an install tree built
    against different paths, which fails in ways that look like code faults."""
    t = (REPO / '.dockerignore').read_text()
    for d in ('build/', 'install/', 'log/'):
        assert d in t
