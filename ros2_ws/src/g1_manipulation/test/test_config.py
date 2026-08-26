from pathlib import Path

import yaml


CONFIG_FILE = Path(__file__).parents[1] / "config" / "manipulation.yaml"


def test_planning_retries_are_enabled():
    config = yaml.safe_load(CONFIG_FILE.read_text(encoding="utf-8"))
    parameters = config["g1_manipulation_server"]["ros__parameters"]

    assert parameters["planning_attempts"] == 3
    assert parameters["planning_time_s"] > 0.0
