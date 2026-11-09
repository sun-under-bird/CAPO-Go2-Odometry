"""Offline diagnosis using recorded ground truth; never feeds the estimator.

Compare the legacy joint-only velocity observation with the complete rigid
body velocity, and distinguish a rolling sphere's geometric bottom from its
instantaneous material contact point. Contact labels below are reconstructed
from recorded poses and collision geometry, not force-sensor measurements.
"""
import argparse
import csv
import hashlib
import json
from pathlib import Path

import mujoco
import numpy as np

from marsdog_model import DEFAULT_SDK, LEGS, load_sdk


def columns(path, names):
    with path.open() as stream:
        return np.array([[float(row[name]) for name in names]
                         for row in csv.DictReader(stream)])


def stats(velocity, truth, mask):
    selected = velocity[mask]
    error = selected - truth[mask]
    return {"samples": int(mask.sum()),
            "mean_forward_m_s": float(selected[:, 0].mean()),
            "mean_forward_error_m_s": float(error[:, 0].mean()),
            "horizontal_rmse_m_s": float(np.sqrt(np.mean(np.sum(error[:, :2]**2, axis=1))))}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", type=Path)
    parser.add_argument("--sdk", type=Path, default=DEFAULT_SDK)
    args = parser.parse_args()
    directory = args.directory.resolve()
    summary = json.loads((directory / "summary.json").read_text())
    _, _, model, names, _, qadr, dadr = load_sdk(args.sdk)
    # Replay the actual compiled model when available, even if source files or
    # the default URDF change after this experiment was recorded.
    model_path = directory / "model.mjb"
    if model_path.is_file():
        manifest = json.loads((directory / "provenance.json").read_text())
        if hashlib.sha256(model_path.read_bytes()).hexdigest() != manifest["compiled_model_sha256"]:
            raise ValueError("Recorded model checksum mismatch")
        model = mujoco.MjModel.from_binary_path(str(model_path))
        joints = [model.joint(name).id for name in names]
        qadr, dadr = model.jnt_qposadr[joints], model.jnt_dofadr[joints]
    inputs = columns(directory / "inputs.csv", ["stamp_ns"] +
                     [name+suffix for suffix in (".q_rad", ".dq_rad_s") for name in names] +
                     ["imu_quat_"+axis for axis in "wxyz"] +
                     ["imu_gyro_"+axis for axis in "xyz"])
    samples = columns(directory / "samples.csv", ["t", "gt_x", "gt_y", "gt_z",
                      "gt_vx", "gt_vy", "capo_vx", "capo_vy"] + ["contact_"+leg for leg in LEGS])
    np.testing.assert_allclose(inputs[:, 0]/1e9, samples[:, 0], atol=1e-8, rtol=0)
    n = len(samples)
    data = mujoco.MjData(model)
    feet = [model.geom(leg+"_foot_collision_0").id for leg in LEGS]
    foot_index = {geom: i for i, geom in enumerate(feet)}
    base = model.body("base_link").id
    contact = np.zeros((n, 4), dtype=bool)
    current = np.zeros((n, 4, 3))
    complete = np.zeros_like(current)
    center_fixed = np.zeros_like(current)
    waist_contribution = np.zeros_like(current)
    centers_world = np.zeros_like(current)
    rotations = np.zeros((n, 3, 3))
    gt = np.zeros((n, 3))
    gt[:, :2] = samples[:, 4:6]
    # Recorded body vx/vy plus differentiated world z determine body vz.
    world_vz = np.gradient(samples[:, 3], samples[:, 0])
    jac, angular = np.zeros((3, model.nv)), np.zeros((3, model.nv))
    for k in range(n):
        data.qpos[:3] = samples[k, 1:4]
        data.qpos[3:7] = inputs[k, 29:33]
        data.qpos[qadr] = inputs[k, 1:15]
        mujoco.mj_fwdPosition(model, data)
        rot = data.xmat[base].reshape(3, 3)
        rotations[k] = rot
        gt[k, 2] = (world_vz[k] - rot[2, :2] @ gt[k, :2]) / rot[2, 2]
        omega = rot @ inputs[k, 33:36]
        dq = inputs[k, 15:29]
        for item in data.contact:
            # Only a penetrating foot/plane pair is used as a contact label.
            for foot, other in ((item.geom1, item.geom2), (item.geom2, item.geom1)):
                if foot in foot_index and model.geom_type[other] == mujoco.mjtGeom.mjGEOM_PLANE and item.dist <= 0:
                    contact[k, foot_index[foot]] = True
        for i, geom in enumerate(feet):
            center = data.geom_xpos[geom].copy()
            centers_world[k, i] = center
            point = center.copy()
            point[2] -= model.geom_size[geom, 0]
            body = int(model.geom_bodyid[geom])
            mujoco.mj_jac(model, data, jac, angular, point, body)
            current[k, i] = (-jac[:, dadr] @ dq) @ rot
            complete[k, i] = (-jac[:, dadr] @ dq - np.cross(omega, point-data.xpos[base])) @ rot
            waist_contribution[k, i] = (-jac[:, dadr[-2:]] @ dq[-2:]) @ rot
            mujoco.mj_jac(model, data, jac, angular, center, body)
            center_fixed[k, i] = (-jac[:, dadr] @ dq - np.cross(omega, center-data.xpos[base])) @ rot
        if (k+1) % 20000 == 0:
            print(f"Reconstructed {k+1}/{n} frames", flush=True)

    # Select whole steady drive intervals, not individual fast gait samples.
    # These are diagnostic averages, not full-trajectory scores.
    warmup_end = samples[n-summary["samples"], 0]
    moving = samples[:, 0] >= warmup_end+2
    selection = "straight run after warmup + 2 seconds"
    if summary["scenario"] == "square":
        with (directory / "samples.csv").open() as stream:
            drive = np.array([r["route_phase"] == "drive" for r in csv.DictReader(stream)])
        edges = np.diff(np.r_[False, drive, False].astype(int))
        moving[:] = False
        for start, end in zip(np.flatnonzero(edges == 1), np.flatnonzero(edges == -1)):
            a = max(samples[start, 0], warmup_end) + 2
            b = samples[end-1, 0] - 5
            moving[start:end] = (samples[start:end, 0] >= a) & (samples[start:end, 0] <= b)
        selection = "each drive phase, excluding first 2 and last 5 seconds; warmup excluded"
    if summary["scenario"] not in ("straight", "square") or not moving.any():
        raise ValueError("Diagnosis requires a sufficiently long straight or square run")
    estimated_contact = samples[:, 8:12] > .5
    result = {"source": str(directory), "purpose": "offline diagnostic only; no estimator modification",
              "contact_reference": "recorded poses reconstructed in MuJoCo: foot/plane distance <= 0",
              "frames": n, "moving_frames": int(moving.sum()), "sample_selection": selection,
              "mean_truth_forward_m_s": float(gt[moving, 0].mean()),
              "mean_capo_forward_m_s": float(samples[moving, 6].mean()),
              "motor_count": len(names), "actuator_count": int(model.nu),
              "waist_angle_range_deg": {name: np.degrees([inputs[:, j+1].min(), inputs[:, j+1].max()]).tolist()
                                        for j, name in enumerate(names) if name.startswith("waist_")},
              "velocity_observations": {}, "per_leg": {}}
    for label, selected in (("estimated_contact", estimated_contact), ("geometry_contact", contact)):
        count = selected.sum(axis=1)
        valid = moving & (count > 0)
        result["velocity_observations"][label] = {}
        for name, value in (("current_joint_only", current), ("with_body_angular_velocity", complete),
                            ("assuming_sphere_center_fixed", center_fixed)):
            mean = (value*selected[:, :, None]).sum(axis=1)/np.maximum(count[:, None], 1)
            result["velocity_observations"][label][name] = stats(mean, gt, valid)
    for i, leg in enumerate(LEGS):
        predicted, actual = estimated_contact[:, i], contact[:, i]
        both = moving & predicted & actual
        # Verify the sphere-center velocity with a separate finite difference.
        center_velocity = np.gradient(centers_world[:, i], samples[:, 0], axis=0)
        center_velocity_body = np.einsum("ni,nij->nj", center_velocity, rotations)
        residual = center_velocity_body - (gt-center_fixed[:, i])
        result["per_leg"][leg] = {
            "predicted_contact_precision": float((predicted & actual & moving).sum()/max(1, (predicted & moving).sum())),
            "predicted_contact_recall": float((predicted & actual & moving).sum()/max(1, (actual & moving).sum())),
            "sphere_bottom_forward_velocity_when_both_contact_m_s": float((gt[both, 0]-center_fixed[both, i, 0]).mean()),
            "material_contact_forward_slip_when_both_contact_m_s": float((gt[both, 0]-complete[both, i, 0]).mean()),
            "waist_velocity_horizontal_rms_m_s": float(np.sqrt(np.mean(np.sum(waist_contribution[moving, i, :2]**2, axis=1)))),
            "center_velocity_finite_difference_horizontal_rmse_m_s": float(np.sqrt(np.mean(np.sum(residual[moving, :2]**2, axis=1))))}
    output = directory / "kinematic_diagnosis.json"
    output.write_text(json.dumps(result, indent=2, ensure_ascii=False)+"\n")
    print(json.dumps(result, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
