import importlib.util
import unittest
from pathlib import Path

from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition


LAUNCH_FILE = Path(__file__).parents[1] / "launch" / "g1_sim.launch.py"


def _load_launch_module():
    spec = importlib.util.spec_from_file_location("g1_sim_launch", LAUNCH_FILE)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


class LaunchDescriptionTests(unittest.TestCase):
    def test_key_launch_arguments_are_declared(self) -> None:
        launch_description = _load_launch_module().generate_launch_description()
        arguments = {
            action.name: action
            for action in launch_description.entities
            if isinstance(action, DeclareLaunchArgument)
        }

        for name in [
            "run_sim",
            "mujoco_scene",
            "run_slam",
            "run_nav2",
            "run_human_collision_avoidance",
            "run_locomotion_diagnostics",
            "run_point_goal",
            "policy_path",
        ]:
            self.assertIn(name, arguments)

        self.assertEqual(arguments["mujoco_scene"].default_value[0].text, "scene_29dof_extreme_lab.xml")
        self.assertEqual(arguments["run_human_collision_avoidance"].default_value[0].text, "true")
        self.assertEqual(arguments["run_locomotion_diagnostics"].default_value[0].text, "true")
        self.assertEqual(arguments["run_waypoint_explorer"].default_value[0].text, "false")

    def test_sim_process_is_conditioned_and_delayed_nodes_exist(self) -> None:
        launch_description = _load_launch_module().generate_launch_description()
        sim_processes = [action for action in launch_description.entities if isinstance(action, ExecuteProcess)]
        timers = [action for action in launch_description.entities if isinstance(action, TimerAction)]

        self.assertEqual(len(sim_processes), 1)
        self.assertIsInstance(sim_processes[0].condition, IfCondition)
        self.assertGreaterEqual(len(timers), 3)
        periods = sorted(float(timer.period) for timer in timers)
        self.assertIn(3.0, periods)
        self.assertIn(7.0, periods)
        self.assertIn(11.0, periods)


if __name__ == "__main__":
    unittest.main()
