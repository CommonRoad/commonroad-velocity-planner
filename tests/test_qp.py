import unittest
from pathlib import Path
import os

# own code base
from run_example import main
from commonroad_velocity_planner.velocity_planner_interface import ImplementedPlanners


class TestQP(unittest.TestCase):
    """
    Test main script
    """

    def test_QP(self):
        not_working = []

        path_scenarios = Path(__file__).parents[1] / "scenarios"

        for idx, filename in enumerate(sorted(os.listdir(path_scenarios))):
            abs_path = os.path.join(path_scenarios, filename)

            if abs_path.split("/")[-1] not in not_working:
                print(f"Testing scenario [{idx} / {len(os.listdir(path_scenarios))}] {abs_path}")
                main(abs_path, test=True, planner=ImplementedPlanners.QPPlanner, output_dir_path="")
