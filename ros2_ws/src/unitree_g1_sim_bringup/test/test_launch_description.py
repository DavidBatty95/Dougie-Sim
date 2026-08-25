import importlib.util
import unittest
from pathlib import Path

import yaml
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile


LAUNCH_FILE = Path(__file__).parents[1] / "launch" / "g1_sim.launch.py"
CONFIG_DIR = Path(__file__).parents[1] / "config"
NAV2_CONFIG = CONFIG_DIR / "nav2_livox.yaml"
PROFILE_NAMES = ["perception_only", "balanced", "conservative", "hardware_safe", "oracle"]
SOCIAL_NODE_NAMES = {
    "dynamic_obstacle_tracker",
    "human_aware_motion_planner",
    "human_collision_avoidance",
}


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
            "social_navigation_params_file",
            "run_locomotion_diagnostics",
            "run_point_goal",
            "run_base_approach",
            "base_approach_params_file",
            "policy_path",
            "policy_profile",
        ]:
            self.assertIn(name, arguments)

        self.assertEqual(arguments["mujoco_scene"].default_value[0].text, "scene_29dof_extreme_lab.xml")
        self.assertEqual(arguments["run_human_collision_avoidance"].default_value[0].text, "true")
        self.assertEqual(arguments["run_lidar_person_detector"].default_value[0].text, "false")
        self.assertEqual(arguments["tracker_publish_candidate_predictions"].default_value[0].text, "false")
        self.assertEqual(arguments["run_locomotion_diagnostics"].default_value[0].text, "true")
        self.assertEqual(arguments["run_waypoint_explorer"].default_value[0].text, "false")
        self.assertEqual(arguments["run_base_approach"].default_value[0].text, "false")
        self.assertEqual(
            arguments["policy_path"].default_value[0].text,
            _load_launch_module().DEFAULT_POLICY_PATH,
        )
        expected_defaults = {
            "cmd_vel_timeout": "0.4",
            "lowstate_timeout": "0.1",
            "odom_timeout": "0.2",
            "cmd_vel_deadband": "0.05",
            "axis_exclusive_cmd_vel": "false",
            "axis_exclusive_yaw_threshold": "0.05",
            "allow_reverse_cmd_vel": "true",
            "startup_current_pose_hold_time": "0.0",
            "startup_blend_time": "0.0",
            "startup_hold_time": "0.0",
            "startup_max_recovery_time": "0.0",
            "policy_action_ramp_time": "0.0",
            "policy_action_rate_limit": "8.0",
            "mujoco_initial_keyframe": "policy_stand",
            "max_lin_vel_y": "0.12",
            "max_ang_vel_z": "0.35",
            "policy_profile": "mjlab",
        }
        for name, expected in expected_defaults.items():
            self.assertEqual(arguments[name].default_value[0].text, expected)
        self.assertEqual(
            arguments["social_navigation_params_file"].default_value[0].text,
            str(CONFIG_DIR / "social_navigation_perception_only_v1.yaml"),
        )

    def test_social_navigation_profiles_select_sources_and_preserve_radius_ordering(self) -> None:
        for profile_name in PROFILE_NAMES:
            profile_path = CONFIG_DIR / f"social_navigation_{profile_name}_v1.yaml"
            with self.subTest(profile=profile_name), profile_path.open(encoding="utf-8") as stream:
                profile = yaml.safe_load(stream)

            self.assertEqual(set(profile), SOCIAL_NODE_NAMES)
            tracker = profile["dynamic_obstacle_tracker"]["ros__parameters"]
            planner = profile["human_aware_motion_planner"]["ros__parameters"]
            safety = profile["human_collision_avoidance"]["ros__parameters"]

            oracle = profile_name == "oracle"
            self.assertEqual(tracker["use_mujoco_people_detections"], oracle)
            self.assertEqual(tracker["mujoco_people_authoritative"], oracle)
            self.assertEqual(safety["use_mujoco_people"], oracle)
            self.assertFalse(planner["use_legacy_person_inputs"])
            self.assertEqual(planner["tracked_persons_topic"], "/tracked_persons")
            self.assertEqual(planner["predicted_trajectories_topic"], "/predicted_person_trajectories")
            self.assertLess(safety["critical_radius"], safety["danger_radius"])
            self.assertLess(safety["emergency_stop_radius"], safety["critical_radius"])

    def test_social_navigation_profile_is_last_parameter_source_for_each_node(self) -> None:
        launch_description = _load_launch_module().generate_launch_description()
        nodes = {
            action._Node__node_name: action
            for timer in launch_description.entities
            if isinstance(timer, TimerAction)
            for action in timer.actions
            if isinstance(action, Node) and action._Node__node_name in SOCIAL_NODE_NAMES
        }

        self.assertEqual(set(nodes), SOCIAL_NODE_NAMES)
        for name, node in nodes.items():
            with self.subTest(node=name):
                final_source = node._Node__parameters[-1]
                self.assertIsInstance(final_source, ParameterFile)
                self.assertIsInstance(final_source.param_file[-1], LaunchConfiguration)
                self.assertEqual(
                    final_source.param_file[-1].variable_name[0].text,
                    "social_navigation_params_file",
                )

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

    def test_policy_node_uses_hard_safety_limits(self) -> None:
        launch_description = _load_launch_module().generate_launch_description()
        policy_node = next(
            action
            for timer in launch_description.entities
            if isinstance(timer, TimerAction)
            for action in timer.actions
            if isinstance(action, Node)
            and action._Node__node_name == "g1_policy_cmd_vel_controller"
        )
        parameters = {
            "".join(part.text for part in key): value
            for key, value in policy_node._Node__parameters[0].items()
        }

        self.assertEqual(parameters["action_clip"], 1.0)
        self.assertLessEqual(parameters["policy_inference_deadline_sec"], 0.05)
        self.assertEqual(parameters["startup_upright_tilt_threshold"], 0.35)
        self.assertEqual(parameters["cmd_vel_deadband_hysteresis"], 0.01)

    def test_prediction_sweeps_are_not_flattened_into_costmaps(self) -> None:
        config = yaml.safe_load(NAV2_CONFIG.read_text(encoding="utf-8"))
        local_layer = config["local_costmap"]["local_costmap"]["ros__parameters"]["obstacle_layer"]
        global_layer = config["global_costmap"]["global_costmap"]["ros__parameters"]["obstacle_layer"]

        self.assertEqual(local_layer["observation_sources"], "scan tracked_obstacles")
        self.assertEqual(global_layer["observation_sources"], "scan tracked_obstacles")
        self.assertNotIn("predicted_obstacles", local_layer)
        self.assertNotIn("predicted_obstacles", global_layer)

    def test_base_approach_is_opt_in_and_uses_the_grove_config(self) -> None:
        launch_description = _load_launch_module().generate_launch_description()
        approach_node = next(
            action
            for timer in launch_description.entities
            if isinstance(timer, TimerAction)
            for action in timer.actions
            if isinstance(action, Node) and action._Node__node_name == "g1_base_approach"
        )

        self.assertIsInstance(approach_node.condition, IfCondition)
        self.assertEqual(approach_node.node_package, "g1_locomotion")
        self.assertEqual(approach_node.node_executable, "g1_base_approach")
        parameter_file = approach_node._Node__parameters[0]
        self.assertIsInstance(parameter_file, ParameterFile)
        self.assertIsInstance(parameter_file.param_file[-1], LaunchConfiguration)
        self.assertEqual(
            parameter_file.param_file[-1].variable_name[0].text,
            "base_approach_params_file",
        )

    def test_nav2_velocity_limits_match_policy_safety_envelope(self) -> None:
        config = yaml.safe_load(NAV2_CONFIG.read_text(encoding="utf-8"))
        controller = config["controller_server"]["ros__parameters"]
        planner = controller["FollowPath"]
        smoother = config["velocity_smoother"]["ros__parameters"]

        self.assertEqual(planner["max_vel_x"], 0.35)
        self.assertEqual(planner["min_vel_x"], 0.0)
        self.assertLessEqual(planner["max_vel_theta"], 0.45)
        self.assertEqual(planner["acc_lim_x"], 0.22)
        self.assertEqual(planner["acc_lim_theta"], 0.35)
        self.assertGreaterEqual(controller["min_x_velocity_threshold"], 0.05)
        self.assertEqual(smoother["max_velocity"], [0.35, 0.0, 0.45])
        self.assertEqual(smoother["min_velocity"], [0.0, 0.0, -0.45])
        self.assertEqual(smoother["max_accel"], [0.22, 0.0, 0.35])
        self.assertEqual(smoother["deadband_velocity"], [0.05, 0.0, 0.05])
        self.assertLessEqual(smoother["velocity_timeout"], 0.4)


if __name__ == "__main__":
    unittest.main()
