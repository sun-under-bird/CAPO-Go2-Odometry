"""Load the delivered Marsdog controller and export static geometry for CAPO."""
import hashlib
import importlib.machinery
import importlib.util
import json
import math
from pathlib import Path
import sys

import mujoco
import numpy as np

LEGS = ("fl", "fr", "rl", "rr")
SLOTS = [0, 1, 2, 4, 5, 6, 8, 9, 10, 12, 13, 14, 3, 7]
DEFAULT_SDK = Path("/home/sunbird/Marsdog_V1.7_Roll2100_SLAM_SDK_v0.2.0")


def load_sdk(directory, urdf=None):
    directory = Path(directory).resolve()
    if sys.version_info[:2] not in ((3, 10), (3, 12)):
        raise RuntimeError("Delivered controller requires CPython 3.10 or 3.12")
    code_dir = directory / "sim"
    if sys.version_info.minor == 12:
        code_dir /= "py312"
    path = code_dir / "marsdog_sim.pyc"
    sys.path.insert(0, str(code_dir))
    loader = importlib.machinery.SourcelessFileLoader("marsdog_delivered_sim", str(path))
    spec = importlib.util.spec_from_loader(loader.name, loader)
    module = importlib.util.module_from_spec(spec)
    loader.exec_module(module)
    config = json.loads((directory / "sim/policy/policy.deployment.json").read_text())
    model, names, actuators, qadr, dadr = module.build(directory / "sim/scene/scene.xml", config)
    if len(names) != 14 or len(set(names)) != 14:
        raise ValueError("Expected 14 distinct Marsdog joints")
    if urdf is not None:
        from marsdog_urdf import build_urdf
        model, actuators, qadr, dadr = build_urdf(urdf, model, names)
    return module, config, model, list(names), np.array(actuators), np.array(qadr), np.array(dadr)


def rpy(quaternion):
    w, x, y, z = quaternion
    return [math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y)),
            math.asin(float(np.clip(2*(w*y-z*x), -1, 1))),
            math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))]


def estimator_geometry(model, names):
    """Use model constants only; no simulated pose, force, or contact state."""
    slots = dict(zip(names, SLOTS))
    base = model.body("base_link").id
    parameters = {"joint_names": names, "joint_slots": SLOTS,
                  "robot_mass_kg": float(model.body_mass.sum()),
                  "gravity_m_s2": float(-model.opt.gravity[2]), "foot_radii": []}
    for leg in LEGS:
        geom = model.geom(f"{leg}_foot_collision_0").id
        if model.geom_type[geom] != mujoco.mjtGeom.mjGEOM_SPHERE:
            raise ValueError("First adaptation supports the delivered spherical feet")
        body = int(model.geom_bodyid[geom])
        chain = []
        while body != base:
            if body == 0:
                raise ValueError("Foot is not descended from base_link")
            chain.append(body)
            body = int(model.body_parentid[body])
        flat = []
        for body in reversed(chain):
            count = int(model.body_jntnum[body])
            if count > 1:
                raise ValueError("Expected at most one hinge per Marsdog link")
            slot, axis, use_torque = -1, -1, 0
            if count:
                joint = int(model.body_jntadr[body])
                name = model.joint(joint).name
                if model.jnt_type[joint] != mujoco.mjtJoint.mjJNT_HINGE or not np.allclose(model.jnt_pos[joint], 0):
                    raise ValueError("Unsupported joint type/origin")
                axis = int(np.argmax(model.jnt_axis[joint]))
                if not np.allclose(model.jnt_axis[joint], np.eye(3)[axis]):
                    raise ValueError("Expected positive cardinal joint axis")
                slot = slots[name]
                use_torque = int(name.startswith(leg + "_"))
            flat.extend(float(v) for v in [slot, axis, use_torque,
                        *model.body_pos[body], *rpy(model.body_quat[body])])
        flat.extend(float(v) for v in [-1, -1, 0, *model.geom_pos[geom], 0, 0, 0])
        parameters["chain_" + leg] = flat
        parameters["foot_radii"].append(float(model.geom_size[geom, 0]))
    imu = model.site("imu").id
    imu_body = int(model.site_bodyid[imu])
    if (model.body_parentid[imu_body] != base or model.body_jntnum[imu_body] or
            not np.allclose(model.body_quat[imu_body], [1, 0, 0, 0]) or
            not np.allclose(model.site_quat[imu], [1, 0, 0, 0])):
        raise ValueError("Expected delivered IMU fixed in body axes")
    parameters["imu_position_body"] = (model.body_pos[imu_body] + model.site_pos[imu]).tolist()
    return parameters


def provenance(directory):
    directory = Path(directory)
    bytecode = "sim/py312/marsdog_sim.pyc" if sys.version_info.minor == 12 else "sim/marsdog_sim.pyc"
    names = ["sim/scene/marsdog_v1.7.xml", "sim/policy/policy.onnx",
             "sim/policy/policy.deployment.json", bytecode]
    return {name: hashlib.sha256((directory / name).read_bytes()).hexdigest() for name in names}
