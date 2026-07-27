import importlib.util
import unittest
from pathlib import Path

from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


LAUNCH_FILE = Path(__file__).parents[1] / "launch" / "person_avoidance_benchmark.launch.py"


class PersonAvoidanceLaunchTests(unittest.TestCase):
    def test_launch_declares_required_arguments_and_node(self) -> None:
        spec = importlib.util.spec_from_file_location("person_avoidance_launch", LAUNCH_FILE)
        module = importlib.util.module_from_spec(spec)
        assert spec.loader is not None
        spec.loader.exec_module(module)
        description = module.generate_launch_description()

        arguments = {entity.name for entity in description.entities if isinstance(entity, DeclareLaunchArgument)}
        self.assertTrue({"config", "mode", "backend", "seed", "people", "goals", "output"}.issubset(arguments))
        nodes = [entity for entity in description.entities if isinstance(entity, Node)]
        self.assertEqual(len(nodes), 1)


if __name__ == "__main__":
    unittest.main()
