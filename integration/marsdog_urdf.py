"""Import the measured URDF without changing its geometry or inertial data."""
import hashlib
from pathlib import Path
import struct
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

DEFAULT_URDF = Path("/home/sunbird/marsdog_v17_source/quadruped.urdf")


def inspect_urdf(path):
    path = Path(path).resolve()
    root = ET.parse(path).getroot()
    links = {link.attrib["name"]: link for link in root.findall("link")}
    joints = root.findall("joint")
    if len(links) != 20 or len(joints) != 19:
        raise ValueError("Expected the measured Marsdog tree with 20 links and 19 joints")
    children = [j.find("child").attrib["link"] for j in joints]
    if len(set(children)) != len(children) or set(links)-set(children) != {"base_link"}:
        raise ValueError("Invalid link tree")
    mass = 0.
    for name, link in links.items():
        inertial = link.find("inertial")
        m = float(inertial.find("mass").attrib["value"])
        a = inertial.find("inertia").attrib
        tensor = np.array([[float(a["i"+i+j] if "i"+i+j in a else a["i"+j+i])
                            for j in "xyz"] for i in "xyz"])
        eig = np.linalg.eigvalsh(tensor)
        if not np.isfinite(tensor).all() or not np.isfinite(m) or m <= 0 or np.any(eig <= 0) or eig[-1] > eig[:2].sum()+1e-10:
            raise ValueError(f"Invalid mass or inertia: {name}")
        mass += m
    meshes = {}
    for mesh in root.findall(".//mesh"):
        filename = mesh.attrib["filename"]
        file = path.parent / filename
        contents = file.read_bytes()
        if len(contents) < 84 or len(contents) != 84+50*struct.unpack("<I", contents[80:84])[0]:
            raise ValueError(f"Invalid binary STL: {filename}")
        meshes[filename] = hashlib.sha256(contents).hexdigest()
    active = [j.attrib["name"] for j in joints if j.attrib["type"] != "fixed"]
    if len(active) != 14:
        raise ValueError("Expected 14 active joints")
    return {"urdf": str(path), "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
            "robot_name": root.attrib["name"], "link_count": len(links),
            "joint_count": len(joints), "active_joint_count": len(active),
            "mass_kg": mass, "mesh_sha256": meshes}


def build_urdf(path, controller_model, names):
    """URDF is authoritative; only missing simulation/control settings use SDK.

    Native MjSpec import avoids a lossy intermediate MJCF serialization. The
    deployment JSON's old foot_collision overrides are deliberately not applied.
    """
    path = Path(path).resolve()
    inspection = inspect_urdf(path)
    root = ET.parse(path).getroot()
    active = {j.attrib["name"] for j in root.findall("joint") if j.attrib["type"] != "fixed"}
    if active != set(names):
        raise ValueError("URDF joints do not match the policy's 14 motor names")
    extension = root.find("mujoco")
    if extension is None:
        extension = ET.SubElement(root, "mujoco")
    compiler = extension.find("compiler")
    if compiler is None:
        compiler = ET.SubElement(extension, "compiler")
    compiler.attrib.update(fusestatic="false", discardvisual="false", balanceinertia="false",
                          strippath="false", meshdir=str(path.parent))
    spec = mujoco.MjSpec.from_string(ET.tostring(root, encoding="unicode"))
    spec.modelname = root.attrib["name"]
    spec.option.timestep = .002
    spec.option.gravity = [0., 0., -9.81]
    spec.body("base_link").add_freejoint(name="root")
    spec.worldbody.add_geom(name="floor", type=mujoco.mjtGeom.mjGEOM_PLANE,
                           size=[0., 0., .05], friction=[1., .005, .0001], rgba=[.3, .35, .4, 1.])
    spec.worldbody.add_light(pos=[0., 0., 3.], dir=[0., 0., -1.])
    # Keep all URDF collision shapes. Disable robot self-contact as in the
    # delivered policy environment; retain robot-ground collision.
    for geom in spec.geoms:
        if geom.name == "floor" or (geom.contype == 0 and geom.conaffinity == 0):
            continue
        geom.contype, geom.conaffinity = 1, 0
        geom.friction = [.6, .005, .0001]
    for name in names:
        source = controller_model.actuator(name).id
        joint = controller_model.joint(name).id
        spec.joint(name).armature = controller_model.dof_armature[controller_model.jnt_dofadr[joint]]
        spec.add_actuator(name=name, target=name, trntype=mujoco.mjtTrn.mjTRN_JOINT,
                          gaintype=mujoco.mjtGain.mjGAIN_FIXED, biastype=mujoco.mjtBias.mjBIAS_AFFINE,
                          gainprm=controller_model.actuator_gainprm[source],
                          biasprm=controller_model.actuator_biasprm[source],
                          forcelimited=1, forcerange=controller_model.actuator_forcerange[source],
                          gear=[1., 0., 0., 0., 0., 0.])
    spec.body("imu_link").add_site(name="imu", size=[.003, .003, .003])
    for name, sensor in (("imu_ang_vel", mujoco.mjtSensor.mjSENS_GYRO),
                         ("slam_acc", mujoco.mjtSensor.mjSENS_ACCELEROMETER)):
        spec.add_sensor(name=name, type=sensor, objtype=mujoco.mjtObj.mjOBJ_SITE, objname="imu")
    spec.add_sensor(name="orientation", type=mujoco.mjtSensor.mjSENS_FRAMEQUAT,
                    objtype=mujoco.mjtObj.mjOBJ_XBODY, objname="base_link")
    model = spec.compile()
    np.testing.assert_allclose(model.body_mass.sum(), inspection["mass_kg"], atol=1e-12)
    joints = [model.joint(name).id for name in names]
    actuators = np.array([model.actuator(name).id for name in names])
    return model, actuators, model.jnt_qposadr[joints].copy(), model.jnt_dofadr[joints].copy()
