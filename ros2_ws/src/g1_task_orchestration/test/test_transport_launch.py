import importlib.util
from pathlib import Path

from launch.actions import IncludeLaunchDescription, TimerAction
from launch_ros.actions import Node
import yaml


LAUNCH_FILE = Path(__file__).parents[1] / "launch" / "transport_server.launch.py"
CONFIG_FILE = Path(__file__).parents[1] / "config" / "transport.yaml"
SIM_LAUNCH_FILE = Path(__file__).parents[1] / "launch" / "transport_sim.launch.py"


def _load(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module.generate_launch_description()


def test_launch_starts_only_the_mission_level_server():
    nodes = [
        action
        for action in _load(LAUNCH_FILE, "transport_server_launch").entities
        if isinstance(action, Node)
    ]
    assert len(nodes) == 1
    assert nodes[0].node_package == "g1_task_orchestration"
    assert nodes[0].node_executable == "g1_transport_server"


def test_simulation_launch_explicitly_enables_simulation_manipulation():
    config = yaml.safe_load(CONFIG_FILE.read_text(encoding="utf-8"))
    parameters = config["g1_transport_server"]["ros__parameters"]

    assert parameters["allow_simulation_manipulation"] is True
    assert parameters["cancel_timeout_s"] > 0.0


def test_mobile_gate_has_exclusive_policy_routing_and_required_servers():
    description = _load(SIM_LAUNCH_FILE, "transport_sim_launch")
    simulation = next(
        action
        for action in description.entities
        if isinstance(action, IncludeLaunchDescription)
    )
    arguments = dict(simulation.launch_arguments)

    assert arguments["mujoco_scene"] == "scene_29dof_transport.xml"
    assert arguments["mujoco_initial_keyframe"] == "transport_stand"
    assert arguments["run_arm_compositor"] == "true"
    assert arguments["run_sim_manipulation_io"] == "true"
    assert arguments["sim_manipulation_frame_id"] == "odom"
    assert arguments["run_policy_controller"] == "true"
    assert arguments["run_hold_controller"] == "false"
    assert arguments["run_base_approach"] == "true"
    assert arguments["run_slam"] == "true"
    assert arguments["run_nav2"] == "true"
    assert arguments["run_human_aware_motion_planner"] == "true"
    assert arguments["run_human_collision_avoidance"] == "true"

    competing_sources = (
        "run_waypoint_patrol",
        "run_waypoint_explorer",
        "run_point_goal",
        "run_frontier_explorer",
    )
    assert all(arguments[name] == "false" for name in competing_sources)
    assert sorted(
        float(action.period)
        for action in description.entities
        if isinstance(action, TimerAction)
    ) == [6.0, 10.0, 14.0]
