#!/usr/bin/env python3
"""Offline validation for the optional Grove-G1 source profile."""

import os
from pathlib import Path
import subprocess
import unittest

from vcstool.commands.import_ import get_repositories


ROOT = Path(__file__).resolve().parents[3]
MANIFEST = ROOT / "integrations" / "grove_g1" / "grove_g1.repos"
SETUP = ROOT / "scripts" / "setup_grove_g1.sh"
PIN = "97bfb9ba13569a593b5e3ecfc5ed1d4208b2b681"
URL = "https://github.com/Adyansh04/grove-g1.git"


class GroveG1ProfileTest(unittest.TestCase):
    def test_manifest_is_valid_and_exactly_pinned(self) -> None:
        with MANIFEST.open(encoding="utf-8") as stream:
            repositories = get_repositories(stream)
        self.assertEqual(
            repositories,
            {"grove-g1": {"type": "git", "url": URL, "version": PIN}},
        )
        self.assertEqual(
            MANIFEST.read_text(encoding="utf-8"),
            "repositories:\n"
            "  grove-g1:\n"
            "    type: git\n"
            f"    url: {URL}\n"
            f"    version: {PIN}\n",
        )

    def test_setup_helper_has_offline_safety_guards(self) -> None:
        self.assertTrue(os.access(SETUP, os.X_OK), "setup helper must be executable")
        subprocess.run(["bash", "-n", str(SETUP)], check=True)
        source = SETUP.read_text(encoding="utf-8")
        for required in (
            PIN,
            URL,
            "status --porcelain --untracked-files=all",
            "merge-base --is-ancestor",
            "checkout --detach",
            "ROS 2 Jazzy standalone stack",
            "not imported into or built by this repository's ROS 2 Humble colcon workspace",
        ):
            self.assertIn(required, source)


if __name__ == "__main__":
    unittest.main()
