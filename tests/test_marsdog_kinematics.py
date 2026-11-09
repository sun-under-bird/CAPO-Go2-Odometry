"""Independent MuJoCo Jacobian/FK checks for the C++ CAPO adaptation."""
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

import mujoco
import numpy as np

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "integration"))
from marsdog_model import DEFAULT_SDK, LEGS, SLOTS, load_sdk, estimator_geometry


class MarsdogKinematicsTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temp = tempfile.TemporaryDirectory()
        cls.probe = Path(cls.temp.name) / "probe"
        subprocess.run(["g++", "-std=c++17", "-O2", "-I", str(REPO),
                        str(REPO / "tests/point_leg_probe.cpp"), "-o", str(cls.probe)], check=True)
        _, cls.policy, cls.model, cls.names, _, cls.qadr, cls.dadr = load_sdk(DEFAULT_SDK)
        cls.geometry = estimator_geometry(cls.model, cls.names)

    @classmethod
    def tearDownClass(cls): cls.temp.cleanup()

    def evaluate(self, cases, radii):
        tokens = []
        for leg, radius in zip(LEGS, radii):
            chain = self.geometry["chain_"+leg]
            tokens.extend([len(chain)//9, radius, *chain])
        tokens.append(len(cases))
        for message, quaternion in cases: tokens.extend([*message, *quaternion])
        result = subprocess.run([str(self.probe)], input=" ".join(map(str, tokens)),
                                text=True, capture_output=True, check=True)
        return np.fromstring(result.stdout, sep=" ").reshape(len(cases), 4, 3, 3)

    def test_positions_velocities_and_force_with_moving_waist(self):
        rng = np.random.default_rng(38)
        cases, expected = [], []
        radii = self.geometry["foot_radii"]
        for _ in range(80):
            data = mujoco.MjData(self.model)
            data.qpos[:3] = [1., -.5, .8]
            quat = rng.normal(size=4); quat /= np.linalg.norm(quat)
            data.qpos[3:7] = quat
            q = np.r_[self.policy["default_joint_pos"], 0, 0] + rng.uniform(-.15, .15, 14)
            q[-2:] = rng.uniform([-.5, 0.], [.5, .4])
            dq = rng.uniform(-1.5, 1.5, 14)
            data.qpos[self.qadr], data.qvel[self.dadr] = q, dq
            mujoco.mj_forward(self.model, data)
            msg = np.zeros(48); msg[SLOTS] = q; msg[np.array(SLOTS)+16] = dq
            msg[32+np.array(SLOTS[-2:])] = rng.uniform(-1000, 1000, 2)
            reference = []
            for i, leg in enumerate(LEGS):
                geom = self.model.geom(leg+"_foot_collision_0").id
                contact = data.geom_xpos[geom].copy(); contact[2] -= radii[i]
                jac, angular = np.zeros((3, self.model.nv)), np.zeros((3, self.model.nv))
                mujoco.mj_jac(self.model, data, jac, angular, contact, int(self.model.geom_bodyid[geom]))
                force = rng.uniform([-3, -3, -30], [3, 3, -10])
                tau = jac[:, self.dadr[3*i:3*i+3]].T @ force
                msg[32+np.array(SLOTS[3*i:3*i+3])] = tau
                reference.append([contact-data.xpos[self.model.body("base_link").id],
                                  jac[:, self.dadr] @ dq, force])
            cases.append((msg, quat)); expected.append(reference)
        actual = self.evaluate(cases, radii); expected = np.array(expected)
        np.testing.assert_allclose(actual[:, :, :2], expected[:, :, :2], atol=2e-10)
        np.testing.assert_allclose(actual[:, :, 2], expected[:, :, 2], atol=5e-3)

    def test_waist_velocity_matches_finite_difference_at_point_endpoint(self):
        data = mujoco.MjData(self.model)
        data.qpos[2] = 1
        q = np.r_[self.policy["default_joint_pos"], .2, .1]
        dq = np.r_[np.zeros(12), .7, -.4]
        data.qpos[self.qadr] = q
        mujoco.mj_forward(self.model, data)
        before = np.array([data.geom_xpos[self.model.geom(l+"_foot_collision_0").id].copy() for l in LEGS])
        eps = 1e-7
        data.qpos[self.qadr] += eps*dq
        mujoco.mj_forward(self.model, data)
        after = np.array([data.geom_xpos[self.model.geom(l+"_foot_collision_0").id].copy() for l in LEGS])
        message = np.zeros(48); message[SLOTS] = q; message[np.array(SLOTS)+16] = dq
        actual = self.evaluate([(message, [1, 0, 0, 0])], [0.]*4)[0, :, 1]
        np.testing.assert_allclose(actual, (after-before)/eps, atol=2e-8)
        self.assertGreater(np.linalg.norm(actual[:2]), .05)
        np.testing.assert_allclose(actual[2:], 0, atol=1e-12)

    def test_invalid_geometry_is_rejected(self):
        # A chain with no independent leg torques must never start an estimator.
        result = subprocess.run([str(self.probe)], input="1 0 -1 -1 0 0 0 0 0 0 0",
                                text=True, capture_output=True)
        self.assertEqual(result.returncode, 2)
        self.assertIn("three independent torques", result.stderr)


if __name__ == "__main__": unittest.main()
