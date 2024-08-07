import unittest
from pathlib import Path
import os
from run_example import main


class TestMain(unittest.TestCase):
    """
    Test main script
    """

    def test_main(self):
        not_working = []

        path_scenarios = Path(__file__).parents[1] / "scenarios"

        for idx, filename in enumerate(sorted(os.listdir(path_scenarios))):
            abs_path = os.path.join(path_scenarios, filename)

            if abs_path.split("/")[-1] not in not_working:
                print(f"Testing scenario [{idx} / {len(os.listdir(path_scenarios))}] {abs_path}")
                main(abs_path, test=True, output_dir_path="")
