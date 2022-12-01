import numpy as np
import os
import reach
from tqdm import tqdm
import unittest
import yaml


class PyIKSolver(reach.IKSolver):
    def getJointNames(self):
        return ['j1', 'j2']

    def solveIK(self, _pose: np.ndarray, _seed: dict):
        return [[1.0, 1.0]]


class PyEvaluator(reach.Evaluator):
    def calculateScore(self, _state: dict):
        return 1.0


class PyTargetPoseGenerator(reach.TargetPoseGenerator):
    def generate(self):
        pts = np.linspace(np.zeros((3,), dtype=np.float64), np.ones((3,), dtype=np.float64), 100, endpoint=True)
        poses = []
        for pt in pts:
            pose = np.eye(4, dtype=np.float64)
            pose[0:3, 3] = pt
            poses.append(pose)

        return poses


class PyDisplay(reach.Display):
    def showEnvironment(self):
        return

    def updateRobotPose(self, _pose: dict):
        return

    def showReachNeighborhood(self, _neighborhood):
        return

    def showResults(self, _results: reach.ReachResult):
        return


class PyLogger(reach.Logger):
    def __init__(self):
        super().__init__()
        self.pbar = tqdm(total=0, position=0, leave=True)
        self.progress = 0

    def setMaxProgress(self, max_progress: int):
        self.progress = 0
        self.pbar = tqdm(total=max_progress, position=0, leave=True)

    def printProgress(self, progress: int):
        self.pbar.update(progress - self.progress)
        self.progress = progress

    def printResults(self, results: reach.ReachResultSummary):
        print(results)

    def print(self, message: str):
        print(message)


class ReachStudyFixture(unittest.TestCase):
    def test_run_reach_study(self):
        config_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), '../test/reach_study.yaml')
        with open(config_file, 'r') as file:
            config = yaml.safe_load(file)

        # Set the ROS package path to two directories up from this file
        os.environ['ROS_PACKAGE_PATH'] = os.path.join(os.path.dirname(os.path.realpath(__file__)), '..', '..')
        reach.runReachStudy(config, '', '', False)

    def test_run_reach_study_pure_python(self):
        ik_solver = PyIKSolver()
        evaluator = PyEvaluator()
        target_pose_generator = PyTargetPoseGenerator()
        display = PyDisplay()
        logger = PyLogger()

        params = reach.Parameters()
        params.radius = 0.4
        params.max_steps = 10
        params.step_improvement_threshold = 0.01

        study = reach.ReachStudy(ik_solver, evaluator, target_pose_generator, display, logger, params)
        study.run()
        study.optimize()

        # Show the results
        db = study.getDatabase()

        # Expect to run for one nominal iteration and one optimization iteration
        self.assertEqual(len(db.results), 2)

        results = db.calculateResults()
        logger.printResults(results)
        self.assertAlmostEqual(results.total_pose_score, 100.0)
        self.assertAlmostEqual(results.reach_percentage, 100.0)
        self.assertAlmostEqual(results.norm_total_pose_score, 100.0)

        display.showEnvironment()

        self.assertEqual(len(db.results[-1]), 100)
        display.showResults(db.results[-1])

    def test_run_reach_study_mixed(self):
        # Load the file
        config_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), '../test/reach_study.yaml')
        with open(config_file, 'r') as file:
            config = yaml.safe_load(file)

        params = reach.Parameters()
        opt_config = config["optimization"]
        params.radius = opt_config["radius"]
        params.max_steps = opt_config["max_steps"]
        params.step_improvement_threshold = opt_config["step_improvement_threshold"]

        os.environ['REACH_PLUGINS'] = 'reach_plugins'
        loader = reach.PluginLoader()
        loader.search_libraries_env = 'REACH_PLUGINS'

        # Load the IK solver from a c++ plugin
        ik_config = config["ik_solver"]
        ik_solver_factory = loader.createIKSolverFactoryInstance(ik_config["name"])
        ik_solver = ik_solver_factory.create(ik_config)

        # Load the evaluator from a c++ plugin
        eval_config = config["evaluator"]
        evaluator_factory = loader.createEvaluatorFactoryInstance(eval_config["name"])
        evaluator = evaluator_factory.create(eval_config)

        # Load the logger from a c++ plugin
        display_config = config["display"]
        display_factory = loader.createDisplayFactoryInstance(display_config["name"])
        display = display_factory.create(display_config)

        # Use Python implementations for the target pose generator and logger
        target_pose_generator = PyTargetPoseGenerator()
        logger = PyLogger()

        study = reach.ReachStudy(ik_solver, evaluator, target_pose_generator, display, logger, params)
        study.run()
        study.optimize()

        # Show the results
        db = study.getDatabase()

        # Should run for one nominal iteration and one optimization iteration
        self.assertEqual(len(db.results), 2)

        results = db.calculateResults()
        logger.printResults(results)
        self.assertAlmostEqual(results.total_pose_score, 0.0)
        self.assertAlmostEqual(results.reach_percentage, 100.0)
        self.assertAlmostEqual(results.norm_total_pose_score, 0.0)

        display.showEnvironment()

        self.assertEqual(len(db.results[-1]), 100)
        display.showResults(db.results[-1])


if __name__ == "__main__":
    unittest.main()
