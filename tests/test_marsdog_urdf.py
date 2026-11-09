"""Check the measured URDF independently of the simulator's import path."""
import unittest
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

import test_marsdog_kinematics as legacy
from marsdog_model import estimator_geometry
from marsdog_urdf import DEFAULT_URDF, build_urdf, inspect_urdf


def rpy_matrix(angles):
    r, p, y = angles
    cr, cp, cy = np.cos([r, p, y])
    sr, sp, sy = np.sin([r, p, y])
    return np.array([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                     [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr], [-sp, cp*sr, cp*cr]])


def origin(element):
    transform = np.eye(4)
    if element is not None:
        transform[:3, 3] = np.fromstring(element.get("xyz", "0 0 0"), sep=" ")
        transform[:3, :3] = rpy_matrix(np.fromstring(element.get("rpy", "0 0 0"), sep=" "))
    return transform


def urdf_fk(root, angles, base):
    transforms = {"base_link": base.copy()}
    pending = list(root.findall("joint"))
    while pending:
        progressed = False
        for joint in pending[:]:
            parent = joint.find("parent").get("link")
            if parent not in transforms:
                continue
            transform = origin(joint.find("origin"))
            if joint.get("type") != "fixed":
                axis = np.fromstring(joint.find("axis").get("xyz"), sep=" ")
                axis /= np.linalg.norm(axis)
                x, y, z = axis
                skew = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
                angle = angles[joint.get("name")]
                transform[:3, :3] = transform[:3, :3] @ (np.eye(3)+np.sin(angle)*skew+(1-np.cos(angle))*(skew@skew))
            transforms[joint.find("child").get("link")] = transforms[parent] @ transform
            pending.remove(joint)
            progressed = True
        if not progressed:
            raise ValueError("Disconnected or cyclic URDF")
    return transforms


class MeasuredMarsdogTest(legacy.MarsdogKinematicsTest):
    @classmethod
    def setUpClass(cls):
        super().setUpClass()
        cls.previous_model = cls.model
        cls.model, _, cls.qadr, cls.dadr = build_urdf(DEFAULT_URDF, cls.model, cls.names)
        cls.geometry = estimator_geometry(cls.model, cls.names)
        cls.urdf = ET.parse(DEFAULT_URDF).getroot()

    def test_all_link_poses_and_foot_centers_match_raw_urdf(self):
        rng = np.random.default_rng(481)
        data = mujoco.MjData(self.model)
        links = self.urdf.findall("link")
        for _ in range(40):
            angles = np.r_[self.policy["default_joint_pos"], 0., .1] + rng.uniform(-.2, .2, 14)
            quat = rng.normal(size=4)
            quat /= np.linalg.norm(quat)
            data.qpos[:3], data.qpos[3:7], data.qpos[self.qadr] = [1., -.5, .7], quat, angles
            mujoco.mj_forward(self.model, data)
            base = np.eye(4)
            base[:3, :3] = data.xmat[self.model.body("base_link").id].reshape(3, 3)
            base[:3, 3] = data.qpos[:3]
            reference = urdf_fk(self.urdf, dict(zip(self.names, angles)), base)
            for link in links:
                name = link.get("name")
                body = self.model.body(name).id
                np.testing.assert_allclose(data.xpos[body], reference[name][:3, 3], atol=2e-12)
                np.testing.assert_allclose(data.xmat[body].reshape(3, 3), reference[name][:3, :3], atol=2e-12)
                if name.endswith("_foot"):
                    collision = link.find("collision")
                    expected = reference[name] @ origin(collision.find("origin"))
                    geom = self.model.geom(collision.get("name")).id
                    np.testing.assert_allclose(data.geom_xpos[geom], expected[:3, 3], atol=2e-12)

    def test_mass_inertia_limits_and_collision_geometry_preserved(self):
        info = inspect_urdf(DEFAULT_URDF)
        self.assertEqual(len(info["mesh_sha256"]), 20)
        self.assertEqual(self.model.nu, 14)
        np.testing.assert_allclose(self.model.body_mass.sum(), info["mass_kg"], atol=1e-12)
        for link in self.urdf.findall("link"):
            body = self.model.body(link.get("name")).id
            inertial = link.find("inertial")
            np.testing.assert_allclose(self.model.body_mass[body], float(inertial.find("mass").get("value")), atol=1e-12)
            np.testing.assert_allclose(self.model.body_ipos[body], origin(inertial.find("origin"))[:3, 3], atol=1e-12)
            a = inertial.find("inertia").attrib
            tensor = np.array([[float(a["i"+i+j] if "i"+i+j in a else a["i"+j+i]) for j in "xyz"] for i in "xyz"])
            r = origin(inertial.find("origin"))[:3, :3]
            rotation = np.zeros(9)
            mujoco.mju_quat2Mat(rotation, self.model.body_iquat[body])
            rotation = rotation.reshape(3, 3)
            expected = r @ tensor @ r.T
            actual = rotation @ np.diag(self.model.body_inertia[body]) @ rotation.T
            # MuJoCo diagonalizes full inertia numerically. Compare the full
            # tensor norm, rather than a relative error on tiny cross terms.
            self.assertLess(np.linalg.norm(actual-expected)/np.linalg.norm(expected), 1e-6)
            for collision in link.findall("collision"):
                geom = self.model.geom(collision.get("name")).id
                np.testing.assert_allclose(self.model.geom_pos[geom], origin(collision.find("origin"))[:3, 3], atol=1e-12)
                shape = list(collision.find("geometry"))[0]
                if shape.tag == "sphere":
                    self.assertEqual(self.model.geom_type[geom], mujoco.mjtGeom.mjGEOM_SPHERE)
                    self.assertAlmostEqual(self.model.geom_size[geom, 0], float(shape.get("radius")))
        for joint in self.urdf.findall("joint"):
            if joint.get("type") == "fixed":
                continue
            idx = self.model.joint(joint.get("name")).id
            np.testing.assert_allclose(self.model.jnt_axis[idx], np.fromstring(joint.find("axis").get("xyz"), sep=" "), atol=1e-12)
            limit = joint.find("limit")
            np.testing.assert_allclose(self.model.jnt_range[idx], [float(limit.get("lower")), float(limit.get("upper"))], atol=1e-12)
            actuator = self.model.actuator(joint.get("name")).id
            np.testing.assert_allclose(self.model.actuator_forcerange[actuator], [-float(limit.get("effort")), float(limit.get("effort"))])

    def test_imu_reports_link_frame_not_principal_inertia_frame(self):
        rng = np.random.default_rng(621)
        data = mujoco.MjData(self.model)
        base = self.model.body("base_link").id
        for _ in range(20):
            quat = rng.normal(size=4)
            quat /= np.linalg.norm(quat)
            data.qpos[:3], data.qpos[3:7] = [0., 0., 2.], quat
            data.qvel[3:6] = rng.uniform(-1, 1, 3)
            mujoco.mj_forward(self.model, data)
            sensed = data.sensor("orientation").data
            self.assertAlmostEqual(abs(float(sensed @ data.xquat[base])), 1., places=12)
            np.testing.assert_allclose(data.sensor("imu_ang_vel").data, data.qvel[3:6], atol=1e-12)


if __name__ == "__main__":
    unittest.main()
