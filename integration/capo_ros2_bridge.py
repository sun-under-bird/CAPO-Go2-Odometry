"""Go2 MuJoCo -> CAPO at 200 Hz using only IMU, encoders, and motor torques.

The NP3O controller reproduces sim_patch/example_cpp/policy_runner.cpp at 50 Hz.
Ground truth is published and logged for evaluation, never used by CAPO or the
controller. Each physics step waits for odometry with the exact sensor stamp.
"""
import argparse
import csv
import json
import math
from pathlib import Path
import time
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
import onnxruntime as ort
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64MultiArray, MultiArrayDimension

REPO = Path(__file__).resolve().parents[1]
DT = 0.005
POLICY_DECIMATION = 4
SDK_LEGS = ("FR", "FL", "RR", "RL")
CAPO_LEGS = ("FL", "FR", "RL", "RR")
JOINTS = ("hip", "thigh", "calf")
DEFAULT_SDK = np.array([-0.1, 0.8, -1.5, 0.1, 0.8, -1.5,
                        -0.1, 1.0, -1.5, 0.1, 1.0, -1.5])


def wrap(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


def yaw_of(quaternion):
    w, x, y, z = quaternion
    return math.atan2(2 * (w*z + x*y), 1 - 2 * (y*y + z*z))


def rotation(quaternion):
    result = np.empty(9)
    mujoco.mju_quat2Mat(result, quaternion)
    return result.reshape(3, 3)


class Policy:
    def __init__(self, path):
        options = ort.SessionOptions()
        options.intra_op_num_threads = 1
        options.inter_op_num_threads = 1
        self.session = ort.InferenceSession(str(path), options,
                                           providers=["CPUExecutionProvider"])
        self.history = np.zeros((1, 10, 45), np.float32)
        self.last_action = np.zeros(12, np.float32)
        self.filtered = np.zeros(12)
        self.first = True
        self.heading = None

    def target(self, q, dq, gyro, quaternion, command, ramp):
        if self.heading is None:
            self.heading = yaw_of(quaternion)
        self.heading += command[2] * DT * POLICY_DECIMATION
        yaw_command = np.clip(0.5 * wrap(self.heading - yaw_of(quaternion)), -1, 1)
        observation = np.concatenate([
            gyro * 0.25, rotation(quaternion).T @ np.array([0., 0., -1.]),
            [command[0] * ramp * 2, command[1] * ramp * 2, yaw_command * 0.25],
            q - DEFAULT_SDK, dq * 0.05, self.last_action,
        ]).astype(np.float32)
        action = self.session.run(["action"], {
            "obs": observation[None], "hist": self.history,
        })[0].reshape(12)
        if not np.isfinite(action).all():
            raise RuntimeError("Policy produced non-finite actions")
        action = np.clip(action, -100, 100)
        self.filtered = 0.8 * action + 0.2 * self.filtered
        scale = np.full(12, 0.25)
        scale[::3] *= 0.5
        target = DEFAULT_SDK + self.filtered * scale
        if self.first:
            self.history[:] = observation
            self.first = False
        self.history[:, :-1] = self.history[:, 1:].copy()
        self.history[:, -1] = observation
        self.last_action = action.copy()
        return target


def load_model(path):
    # Resolve include/mesh paths so the model can live outside the checkout.
    scene = ET.parse(path).getroot()
    include = scene.find("include")
    robot = ET.parse(path.parent / include.attrib["file"]).getroot()
    scene.remove(include)
    # The upstream scene contains hurdles and a staircase. These runs measure
    # flat-ground odometry, as in sim_patch/go2_scene/scene.xml.
    for geom in list(scene.find("worldbody").findall("geom")):
        if geom.get("name") != "floor":
            scene.find("worldbody").remove(geom)
    robot.find("compiler").set("meshdir", str(path.parent / "assets"))
    for child in list(scene):
        existing = robot.find(child.tag)
        if existing is not None:
            existing.extend(list(child))
        else:
            robot.append(child)
    robot.find("option").set("timestep", str(DT))
    model = mujoco.MjModel.from_xml_string(ET.tostring(robot, encoding="unicode"))
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, 0)
    return model, data


class Bridge(Node):
    def __init__(self, args):
        super().__init__("capo_mujoco_torque_bridge")
        self.args = args
        self.model, self.data = load_model(args.scene)
        self.base_id = self.model.body("base_link").id
        self.joint_ids = [self.model.joint(f"{leg}_{joint}_joint").id
                          for leg in SDK_LEGS for joint in JOINTS]
        self.qadr = self.model.jnt_qposadr[self.joint_ids]
        self.dadr = self.model.jnt_dofadr[self.joint_ids]
        self.actuator_ids = [self.model.actuator(f"{leg}_{joint}").id
                             for leg in SDK_LEGS for joint in JOINTS]
        self.sensor_slices = {}
        for index in range(self.model.nsensor):
            name = self.model.sensor(index).name
            start = self.model.sensor_adr[index]
            self.sensor_slices[name] = slice(start, start + self.model.sensor_dim[index])
        self.data.qpos[self.qadr] = DEFAULT_SDK
        self.data.qpos[0:2] = [1.0, -0.5]
        self.data.qpos[3:7] = [math.cos(args.initial_yaw/2), 0., 0.,
                               math.sin(args.initial_yaw/2)]
        mujoco.mj_forward(self.model, self.data)
        # Place the geometric feet just above the plane to avoid a drop at start.
        feet = [self.model.body(f"{leg}_foot").id for leg in SDK_LEGS]
        self.data.qpos[2] -= min(self.data.xpos[feet, 2]) - 0.027
        mujoco.mj_forward(self.model, self.data)
        self.target = DEFAULT_SDK.copy()
        self.policy = None if args.scenario == "stand" else Policy(args.policy)
        self.clock_pub = self.create_publisher(Clock, "/clock", 10)
        self.imu_pub = self.create_publisher(Imu, "/SMX/Go2IMU", 10)
        self.joint_pub = self.create_publisher(Float64MultiArray, "/SMX/Go2Joint", 10)
        self.truth_pub = self.create_publisher(Odometry, "/sim/ground_truth", 10)
        self.odom = {}
        self.odom3d = {}
        self.contacts = {}
        self.odom_sub = self.create_subscription(Odometry, "/SMX/Odom_2D",
                                                 self.receive_odom, 10)
        self.odom3d_sub = self.create_subscription(Odometry, "/SMX/Odom",
                                                   self.receive_odom3d, 10)
        self.contact_sub = self.create_subscription(Float64MultiArray,
            "/SMX/ContactEstimate", self.receive_contacts, 10)
        self.rows = []
        self.failure = None

    def sensor(self, name):
        return self.data.sensordata[self.sensor_slices[name]].copy()

    def sdk_joint_data(self, suffix):
        return np.array([self.sensor(f"{leg}_{joint}_{suffix}")[0]
                         for leg in SDK_LEGS for joint in JOINTS])

    def receive_odom(self, message):
        stamp = message.header.stamp.sec * 10**9 + message.header.stamp.nanosec
        self.odom[stamp] = message

    def receive_contacts(self, message):
        stamp = int(message.layout.dim[0].label.split("=", 1)[1])
        self.contacts[stamp] = np.array(message.data)

    def receive_odom3d(self, message):
        stamp = message.header.stamp.sec * 10**9 + message.header.stamp.nanosec
        self.odom3d[stamp] = message

    def wait_for_estimator(self):
        deadline = time.monotonic() + 30
        while self.imu_pub.get_subscription_count() == 0 or self.joint_pub.get_subscription_count() == 0:
            if time.monotonic() > deadline:
                raise RuntimeError("CAPO is not subscribed; start go2_capo_torque_sim.launch.py")
            rclpy.spin_once(self, timeout_sec=0.1)

    def publish_step(self):
        stamp_ns = round(self.data.time * 10**9)
        clock = Clock()
        clock.clock.sec, clock.clock.nanosec = divmod(stamp_ns, 10**9)
        self.clock_pub.publish(clock)
        imu = Imu()
        imu.header.stamp = clock.clock
        imu.header.frame_id = "base_imu"
        w, x, y, z = self.sensor("imu_quat")
        imu.orientation.w, imu.orientation.x = float(w), float(x)
        imu.orientation.y, imu.orientation.z = float(y), float(z)
        gyro, acceleration = self.sensor("imu_gyro"), self.sensor("imu_acc")
        imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z = gyro.tolist()
        imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z = acceleration.tolist()
        joint = Float64MultiArray()
        dimension = MultiArrayDimension()
        dimension.label = f"imu_stamp_ns={stamp_ns}"
        dimension.size, dimension.stride = 48, 48
        joint.layout.dim = [dimension]
        payload = np.zeros(48)
        for leg_index, leg in enumerate(CAPO_LEGS):
            for joint_index, name in enumerate(JOINTS):
                destination = 4 * leg_index + joint_index
                payload[destination] = self.sensor(f"{leg}_{name}_pos")[0]
                payload[16 + destination] = self.sensor(f"{leg}_{name}_vel")[0]
                # Actual joint actuator force, not a contact force or commanded target.
                payload[32 + destination] = self.sensor(f"{leg}_{name}_torque")[0]
        joint.data = payload.tolist()
        truth = Odometry()
        truth.header.stamp = clock.clock
        truth.header.frame_id, truth.child_frame_id = "sim_world", "base_link"
        position = self.data.xpos[self.base_id].copy()
        quaternion = self.data.xquat[self.base_id].copy()
        truth.pose.pose.position.x, truth.pose.pose.position.y, truth.pose.pose.position.z = position.tolist()
        truth.pose.pose.orientation.w, truth.pose.pose.orientation.x, truth.pose.pose.orientation.y, truth.pose.pose.orientation.z = quaternion.tolist()
        velocity = rotation(quaternion).T @ self.data.qvel[:3]
        truth.twist.twist.linear.x, truth.twist.twist.linear.y, truth.twist.twist.linear.z = velocity.tolist()
        truth.twist.twist.angular.x, truth.twist.twist.angular.y, truth.twist.twist.angular.z = self.data.qvel[3:6].tolist()
        # Separate publisher/subscriber scheduling can reorder these messages;
        # CAPO matches the dimension stamp before performing an update.
        self.imu_pub.publish(imu)
        self.joint_pub.publish(joint)
        self.truth_pub.publish(truth)
        deadline = time.monotonic() + 5
        while stamp_ns not in self.odom or stamp_ns not in self.odom3d or stamp_ns not in self.contacts:
            if time.monotonic() > deadline:
                raise RuntimeError(f"Missing CAPO update at simulation t={self.data.time:.3f}")
            rclpy.spin_once(self, timeout_sec=0.02)
        odom = self.odom.pop(stamp_ns)
        odom3d = self.odom3d.pop(stamp_ns)
        contacts = self.contacts.pop(stamp_ns)
        p, q, v = odom.pose.pose.position, odom.pose.pose.orientation, odom.twist.twist.linear
        e_yaw = yaw_of([q.w, q.x, q.y, q.z])
        self.rows.append([self.data.time, p.x, p.y, e_yaw,
                          *position, yaw_of(quaternion), v.x, v.y,
                          *velocity[:2], *contacts, *payload[32:], odom3d.pose.pose.position.z])

    def run(self):
        self.wait_for_estimator()
        viewer = None
        if self.args.viewer:
            from mujoco import viewer as mj_viewer
            viewer = mj_viewer.launch_passive(self.model, self.data)
            viewer.cam.distance = 2.2
            viewer.cam.azimuth = 135
            viewer.cam.elevation = -20
        start = time.monotonic()
        steps = round((self.args.warmup + self.args.duration) / DT)
        try:
            for step in range(steps):
                if viewer is not None and not viewer.is_running():
                    break
                t = self.data.time
                if self.policy is not None and t >= 3.0 and step % POLICY_DECIMATION == 0:
                    scenario_time = max(0., t - self.args.warmup)
                    command = {"straight": [self.args.vx, 0, 0],
                               "turn": [0, 0, self.args.wz],
                               "curve": [self.args.vx, 0, self.args.wz]}[self.args.scenario]
                    if t < self.args.warmup:
                        command = [0, 0, 0]
                    ramp = min(scenario_time / 3.0, 1.)
                    self.target = self.policy.target(
                        self.sdk_joint_data("pos"), self.sdk_joint_data("vel"),
                        self.sensor("imu_gyro"), self.sensor("imu_quat"), command, ramp)
                torque = 40.0 * (self.target - self.data.qpos[self.qadr]) - self.data.qvel[self.dadr]
                limits = self.model.actuator_ctrlrange[self.actuator_ids]
                self.data.ctrl[self.actuator_ids] = np.clip(torque, limits[:, 0], limits[:, 1])
                mujoco.mj_step(self.model, self.data)
                mujoco.mj_forward(self.model, self.data)
                self.publish_step()
                height = self.data.xpos[self.base_id, 2]
                upright = rotation(self.data.xquat[self.base_id])[2, 2]
                if not np.isfinite(self.data.qpos).all() or height < 0.14 or upright < 0.4:
                    raise RuntimeError(f"Go2 fell at t={self.data.time:.2f}, height={height:.3f}")
                if viewer is not None and step % POLICY_DECIMATION == 0:
                    viewer.cam.lookat[:] = self.data.xpos[self.base_id]
                    viewer.sync()
                if self.args.realtime:
                    remainder = start + self.data.time - time.monotonic()
                    if remainder > 0:
                        time.sleep(remainder)
                if step % 1000 == 999:
                    print(f"t={self.data.time:.1f}s base_z={height:.3f} frames={len(self.rows)}", flush=True)
        except BaseException as error:
            self.failure = str(error) or type(error).__name__
            raise
        finally:
            if viewer is not None:
                viewer.close()
            self.write_results(time.monotonic() - start)

    def write_results(self, wall_time):
        directory = self.args.output
        directory.mkdir(parents=True, exist_ok=True)
        header = ["t", "capo_x", "capo_y", "capo_yaw", "gt_x", "gt_y", "gt_z", "gt_yaw",
                  "capo_vx_body", "capo_vy_body", "gt_vx_body", "gt_vy_body"]
        header += [f"fz_{leg}" for leg in CAPO_LEGS]
        header += [f"contact_{leg}" for leg in CAPO_LEGS]
        header += [f"probability_{leg}" for leg in CAPO_LEGS]
        header += [f"tau_{leg}_{joint}" for leg in CAPO_LEGS
                   for joint in (*JOINTS, "wheel")]
        header += ["capo_z"]
        with (directory / "samples.csv").open("w") as stream:
            writer = csv.writer(stream)
            writer.writerow(header)
            writer.writerows(self.rows)
        if len(self.rows) < 2:
            return
        all_samples = np.array(self.rows)
        samples = all_samples[all_samples[:, 0] >= self.args.warmup]
        if len(samples) < 2:
            return
        estimated, truth = samples[:, 1:3], samples[:, 4:6]
        angle = samples[0, 3] - samples[0, 7]
        transform = np.array([[math.cos(angle), -math.sin(angle)],
                              [math.sin(angle), math.cos(angle)]])
        aligned = (truth - truth[0]) @ transform.T + estimated[0]
        error = estimated - aligned
        yaw_error = (samples[:, 3] - samples[0, 3]) - (samples[:, 7] - samples[0, 7])
        yaw_error = (yaw_error + math.pi) % (2 * math.pi) - math.pi
        path_length = np.linalg.norm(np.diff(truth, axis=0), axis=1).sum()
        summary = {
            "completed": self.failure is None and len(self.rows) == steps_expected(self.args),
            "failure": self.failure,
            "scenario": self.args.scenario, "duration_s": float(samples[-1, 0] - samples[0, 0]),
            "samples": len(samples), "all_frames": len(self.rows),
            "simulation_hz": float(1.0 / np.mean(np.diff(all_samples[:, 0]))),
            "wall_time_s": wall_time, "ground_truth_path_m": float(path_length),
            "gt_displacement_m": float(np.linalg.norm(truth[-1] - truth[0])),
            "capo_displacement_m": float(np.linalg.norm(estimated[-1] - estimated[0])),
            "endpoint_error_m": float(np.linalg.norm(error[-1])),
            "endpoint_error_percent_of_path": float(100 * np.linalg.norm(error[-1]) / max(path_length, 1e-6)),
            "position_rmse_m": float(np.sqrt(np.mean(np.sum(error**2, axis=1)))),
            "yaw_rmse_rad": float(np.sqrt(np.mean(yaw_error**2))),
            "gt_yaw_change_deg": float(np.degrees(np.unwrap(samples[:, 7])[-1] - np.unwrap(samples[:, 7])[0])),
            "capo_yaw_change_deg": float(np.degrees(np.unwrap(samples[:, 3])[-1] - np.unwrap(samples[:, 3])[0])),
            "velocity_rmse_m_s": float(np.sqrt(np.mean(np.sum((samples[:, 8:10] - samples[:, 10:12])**2, axis=1)))),
            "minimum_base_height_m": float(samples[:, 6].min()),
            "height_rmse_m": float(np.sqrt(np.mean((samples[:, -1] - samples[:, 6])**2))),
            "mean_equivalent_fz_N": samples[:, 12:16].mean(axis=0).tolist(),
            "contact_fraction": samples[:, 16:20].mean(axis=0).tolist(),
            "foot_sensor_input": False, "joint_torque_source": "MuJoCo jointactuatorfrc",
            "foot_force_threshold_N": self.args.foot_threshold,
            "initial_yaw_rad": self.args.initial_yaw,
            "alignment": "initial SE(2) pose only; no scale fitting",
        }
        (directory / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
        print(json.dumps(summary, indent=2), flush=True)
        import matplotlib
        matplotlib.use("Agg")
        # Ubuntu's namespace .pth can pre-load an older system mpl_toolkits.
        # Resolve the matching toolkit within this process's virtual environment.
        import mpl_toolkits
        toolkit = Path(matplotlib.__file__).parent.parent / "mpl_toolkits"
        if toolkit.is_dir():
            mpl_toolkits.__path__ = [str(toolkit)]
        import matplotlib.pyplot as plt
        figure, axes = plt.subplots(2, 2, figsize=(12, 8), constrained_layout=True)
        axes[0, 0].plot(aligned[:, 0], aligned[:, 1], label="MuJoCo (initial pose aligned)")
        axes[0, 0].plot(estimated[:, 0], estimated[:, 1], label="CAPO (joint torque)")
        axes[0, 0].axis("equal")
        axes[0, 0].set(xlabel="x [m]", ylabel="y [m]", title=self.args.scenario)
        axes[0, 0].legend()
        axes[0, 1].plot(samples[:, 0], np.linalg.norm(error, axis=1))
        axes[0, 1].set(xlabel="Simulation time [s]", ylabel="Position error [m]")
        for leg_index, leg in enumerate(CAPO_LEGS):
            axes[1, 0].plot(samples[:, 0], samples[:, 12+leg_index], label=leg, alpha=0.7)
        axes[1, 0].axhline(self.args.foot_threshold, color="black", linestyle="--", label="Threshold")
        axes[1, 0].set(xlabel="Simulation time [s]", ylabel="Equivalent foot Fz [N]")
        axes[1, 0].legend()
        axes[1, 1].plot(samples[:, 0], np.degrees(yaw_error))
        axes[1, 1].set(xlabel="Simulation time [s]", ylabel="Relative yaw error [deg]")
        figure.savefig(directory / "comparison.png", dpi=160)
        plt.close(figure)


def steps_expected(args):
    return round((args.warmup + args.duration) / DT)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene", type=Path, default=REPO / ".runtime/go2/scene.xml")
    parser.add_argument("--policy", type=Path, default=REPO / "sim_patch/models/go2_policy.onnx")
    parser.add_argument("--scenario", choices=["stand", "straight", "turn", "curve"], default="straight")
    parser.add_argument("--duration", type=float, default=20.)
    parser.add_argument("--warmup", type=float, default=6.)
    parser.add_argument("--vx", type=float, default=0.4)
    parser.add_argument("--wz", type=float, default=0.4)
    parser.add_argument("--initial-yaw", type=float, default=0.7)
    parser.add_argument("--foot-threshold", type=float, default=-20.)
    parser.add_argument("--viewer", action="store_true")
    parser.add_argument("--realtime", action="store_true")
    parser.add_argument("--output", type=Path, default=REPO / ".runtime/results/latest")
    args = parser.parse_args()
    if args.duration <= 0 or args.warmup < 3 or abs(args.vx) > 0.5 or abs(args.wz) > 1 or args.foot_threshold >= 0:
        parser.error("duration > 0, warmup >= 3, |vx| <= 0.5, |wz| <= 1, foot threshold < 0 are required")
    rclpy.init(args=[])
    node = None
    try:
        node = Bridge(args)
        node.run()
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
