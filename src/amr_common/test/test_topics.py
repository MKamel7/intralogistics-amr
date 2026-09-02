"""The topic names in the YAML and the ones in the code must be the same names.

This is the check that makes the constant worth having. Without it, renaming
the controller in `controllers.*.yaml` leaves five Python files subscribing to
a topic that no longer exists, and nothing anywhere says so: a subscriber to a
dead topic never errors, it just goes quiet.
"""

from pathlib import Path

import pytest
import yaml
from amr_common.topics import CONTROLLER, Topics

SRC = Path(__file__).resolve().parents[2]
CONTROLLER_CONFIGS = sorted((SRC / "amr_description" / "config").glob("controllers.*.yaml"))
MONITOR_CONFIGS = sorted((SRC / "amr_safety" / "config").glob("collision_monitor.*.yaml"))


def test_the_configs_this_gate_reads_exist():
    """A gate that silently finds no files is the failure mode it exists to stop."""
    assert CONTROLLER_CONFIGS, "no controller configs found, so nothing was checked"
    assert MONITOR_CONFIGS, "no collision monitor configs found, so nothing was checked"


@pytest.mark.parametrize("config", CONTROLLER_CONFIGS, ids=lambda p: p.name)
def test_every_platform_declares_the_controller_this_code_talks_to(config):
    loaded = yaml.safe_load(config.read_text(encoding="utf-8"))

    assert CONTROLLER in loaded, (
        f"{config.name} has no '{CONTROLLER}' section, so amr_common.topics is "
        f"naming a controller this platform does not spawn")
    spawned = loaded["controller_manager"]["ros__parameters"]
    assert CONTROLLER in spawned, f"{config.name} does not spawn {CONTROLLER}"


@pytest.mark.parametrize("config", MONITOR_CONFIGS, ids=lambda p: p.name)
def test_the_safety_layer_sits_between_the_topics_the_code_names(config):
    """cmd_vel_raw in, the controller's own topic out, with the monitor between."""
    loaded = yaml.safe_load(config.read_text(encoding="utf-8"))
    params = loaded["collision_monitor"]["ros__parameters"]

    assert "/" + params["cmd_vel_in_topic"] == Topics.CMD_VEL_RAW
    assert "/" + params["cmd_vel_out_topic"] == Topics.CONTROLLER_CMD_VEL


def test_the_derived_names_follow_the_controller_constant():
    """Rename the controller and the derived topics must move with it."""
    assert Topics.ODOM == f"/{CONTROLLER}/odom"
    assert Topics.CONTROLLER_CMD_VEL == f"/{CONTROLLER}/cmd_vel"
