"""Marsdog sensors -> CAPO; square route navigation may use simulation pose."""
import argparse
import csv
import json
import math
from pathlib import Path
import time

import mujoco
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float64MultiArray

from marsdog_model import LEGS, load_sdk
from square_route import SquareRoute
from capo_ros2_bridge import rotation, yaw_of


class MarsdogBridge(Node):
    def __init__(self, args):
        super().__init__("marsdog_capo_sim_bridge")
        self.args = args
        sdk, config, self.model, self.names, self.act, self.qadr, self.dadr = load_sdk(args.sdk, args.urdf)
        self.actor = sdk.Actor(config, args.sdk / "sim/policy/policy.onnx")
        self.dt = float(self.model.opt.timestep)
        self.decimation = round(config["policy_dt"] / self.dt)
        if not math.isclose(self.decimation * self.dt, config["policy_dt"]):
            raise ValueError("Physics timestep must divide the policy interval")
        self.data = mujoco.MjData(self.model)
        self.data.qpos[:3] = [1.0, -0.5, 0.278]
        self.data.qpos[3:7] = [math.cos(args.initial_yaw/2), 0, 0, math.sin(args.initial_yaw/2)]
        self.data.qpos[self.qadr] = np.r_[config["default_joint_pos"], 0, 0]
        self.data.ctrl[self.act] = self.data.qpos[self.qadr]
        mujoco.mj_forward(self.model, self.data)
        self.base = self.model.body("base_link").id
        self.clock_pub = self.create_publisher(Clock, "/clock", 10)
        self.imu_pub = self.create_publisher(Imu, "/marsdog/imu", 10)
        self.joint_pub = self.create_publisher(JointState, "/marsdog/joint_states", 10)
        self.truth_pub = self.create_publisher(Odometry, "/sim/ground_truth", 10)
        self.odom, self.odom3d, self.contacts = {}, {}, {}
        self.odom_sub = self.create_subscription(Odometry, "/SMX/Odom_2D", self.receive_odom, 10)
        self.odom3d_sub = self.create_subscription(Odometry, "/SMX/Odom", self.receive_odom3d, 10)
        self.contact_sub = self.create_subscription(Float64MultiArray, "/SMX/ContactEstimate", self.receive_contacts, 10)
        self.rows = []
        self.route = SquareRoute(args.side_length, args.vx) if args.scenario == "square" else None
        self.waist_ranges = {name: [float("inf"), -float("inf")] for name in self.names[-2:]}
        args.output.mkdir(parents=True, exist_ok=True)
        self.input_file = (args.output / "inputs.csv").open("w")
        self.input_writer = None
        self.failure = None

    @staticmethod
    def stamp(msg):
        return msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec

    def receive_odom(self, msg): self.odom[self.stamp(msg)] = msg
    def receive_odom3d(self, msg): self.odom3d[self.stamp(msg)] = msg
    def receive_contacts(self, msg):
        self.contacts[int(msg.layout.dim[0].label.split("=", 1)[1])] = np.array(msg.data)

    def sensor(self, name): return self.data.sensor(name).data.copy()

    def publish_step(self):
        stamp = round(self.data.time * 10**9)
        clock = Clock()
        clock.clock.sec, clock.clock.nanosec = divmod(stamp, 10**9)
        self.clock_pub.publish(clock)
        imu = Imu()
        imu.header.stamp, imu.header.frame_id = clock.clock, "imu_link"
        quat, gyro, acc = self.sensor("orientation"), self.sensor("imu_ang_vel"), self.sensor("slam_acc")
        imu.orientation.w, imu.orientation.x, imu.orientation.y, imu.orientation.z = quat.tolist()
        imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z = gyro.tolist()
        imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z = acc.tolist()
        joint = JointState()
        joint.header.stamp, joint.header.frame_id = clock.clock, "base_link"
        joint.name = self.names
        joint.position = self.data.qpos[self.qadr].tolist()
        joint.velocity = self.data.qvel[self.dadr].tolist()
        # General actuators in the delivered model have unit gear. Reading
        # generalized actuator torque excludes contact/constraint forces.
        joint.effort = self.data.qfrc_actuator[self.dadr].tolist()
        self.imu_pub.publish(imu)
        self.joint_pub.publish(joint)

        # Pose is evaluation-only here; SquareRoute separately uses position
        # for navigation. It is never an input to the CAPO estimator.
        position, quaternion = self.data.xpos[self.base].copy(), self.data.xquat[self.base].copy()
        velocity = rotation(quaternion).T @ self.data.qvel[:3]
        truth = Odometry()
        truth.header.stamp, truth.header.frame_id, truth.child_frame_id = clock.clock, "sim_world", "base_link"
        truth.pose.pose.position.x, truth.pose.pose.position.y, truth.pose.pose.position.z = position.tolist()
        truth.pose.pose.orientation.w, truth.pose.pose.orientation.x, truth.pose.pose.orientation.y, truth.pose.pose.orientation.z = quaternion.tolist()
        truth.twist.twist.linear.x, truth.twist.twist.linear.y, truth.twist.twist.linear.z = velocity.tolist()
        truth.twist.twist.angular.x, truth.twist.twist.angular.y, truth.twist.twist.angular.z = self.data.qvel[3:6].tolist()
        self.truth_pub.publish(truth)
        deadline = time.monotonic() + 5
        while stamp not in self.odom or stamp not in self.odom3d or stamp not in self.contacts:
            if time.monotonic() > deadline:
                raise RuntimeError(f"Missing CAPO update at {self.data.time:.3f}s; inspect capo.log")
            rclpy.spin_once(self, timeout_sec=0.01)
        odom, odom3d, contact = self.odom.pop(stamp), self.odom3d.pop(stamp), self.contacts.pop(stamp)
        p, q, v = odom.pose.pose.position, odom.pose.pose.orientation, odom.twist.twist.linear
        row = dict(t=self.data.time, capo_x=p.x, capo_y=p.y, capo_z=odom3d.pose.pose.position.z,
                   capo_yaw=yaw_of([q.w, q.x, q.y, q.z]), gt_x=position[0], gt_y=position[1], gt_z=position[2],
                   gt_yaw=yaw_of(quaternion), capo_vx=v.x, capo_vy=v.y, gt_vx=velocity[0], gt_vy=velocity[1])
        for i, leg in enumerate(LEGS):
            row.update({"fz_"+leg: contact[i], "contact_"+leg: contact[4+i], "probability_"+leg: contact[8+i]})
        if not all(math.isfinite(x) for x in row.values()):
            raise RuntimeError("Non-finite odometry or contact estimate")
        if self.route is not None:
            row.update(route_side=min(self.route.side+1, 4), route_phase=self.route.phase)
        self.rows.append(row)
        sample = {"stamp_ns": stamp}
        for i, name in enumerate(self.names):
            sample.update({name+".q_rad": joint.position[i], name+".dq_rad_s": joint.velocity[i],
                           name+".tau_nm": joint.effort[i]})
        sample.update({"imu_quat_"+axis: value for axis, value in zip("wxyz", quat)})
        sample.update({"imu_gyro_"+axis: value for axis, value in zip("xyz", gyro)})
        sample.update({"imu_acc_"+axis: value for axis, value in zip("xyz", acc)})
        if self.input_writer is None:
            self.input_writer = csv.DictWriter(self.input_file, fieldnames=list(sample))
            self.input_writer.writeheader()
        self.input_writer.writerow(sample)
        for name, limits in self.waist_ranges.items():
            value = sample[name+".q_rad"]
            limits[0], limits[1] = min(limits[0], value), max(limits[1], value)

    def run(self):
        viewer = None
        start = time.monotonic()
        try:
            deadline = start + 30
            while self.imu_pub.get_subscription_count() == 0 or self.joint_pub.get_subscription_count() == 0:
                if time.monotonic() > deadline:
                    raise RuntimeError("CAPO subscriptions missing; inspect capo.log")
                rclpy.spin_once(self, timeout_sec=0.1)
            if self.args.viewer:
                from mujoco import viewer as mj_viewer
                viewer = mj_viewer.launch_passive(self.model, self.data)
                viewer.cam.distance, viewer.cam.azimuth, viewer.cam.elevation = 1.8, 135, -20
            start = time.monotonic()
            duration = self.args.duration
            if self.route is not None:
                duration = self.args.square_timeout or (4*self.args.side_length/.12 + 100.)
            self.expected = round((self.args.warmup + duration) / self.dt)
            for step in range(self.expected):
                if viewer is not None and not viewer.is_running():
                    raise RuntimeError("Viewer closed before the run completed")
                if step % self.decimation == 0:
                    t = self.data.time - self.args.warmup
                    if self.route is not None:
                        command = np.zeros(3) if t < -1e-9 else self.route.update(
                            self.data.time, self.data.xpos[self.base, :2], yaw_of(self.sensor("orientation")))
                        if self.route.done: break
                    else:
                        command = {"stand": [0., 0., 0.], "straight": [self.args.vx, 0., 0.],
                                   "turn": [0., 0., self.args.wz]}[self.args.scenario]
                        if t < 0: command = [0., 0., 0.]
                    gravity = rotation(self.sensor("orientation")).T @ np.array([0., 0., -1.])
                    target = self.actor.step(self.data.qpos[self.qadr], self.data.qvel[self.dadr],
                                             self.sensor("imu_ang_vel"), gravity, np.array(command))
                    if not np.isfinite(target).all(): raise RuntimeError("Non-finite policy target")
                    self.data.ctrl[self.act] = target
                mujoco.mj_step(self.model, self.data)
                mujoco.mj_forward(self.model, self.data)
                self.publish_step()
                height = self.data.xpos[self.base, 2]
                if not np.isfinite(self.data.qpos).all() or height < 0.14 or rotation(self.data.xquat[self.base])[2, 2] < 0.4:
                    raise RuntimeError(f"Marsdog fell at t={self.data.time:.2f}s")
                if viewer is not None and step % self.decimation == 0:
                    viewer.cam.lookat[:] = self.data.xpos[self.base]
                    viewer.sync()
                if self.args.realtime:
                    delay = start + self.data.time - time.monotonic()
                    if delay > 0: time.sleep(delay)
                if (step+1) % 2500 == 0:
                    progress = "" if self.route is None else f" side={self.route.side+1}/4 phase={self.route.phase}"
                    print(f"t={self.data.time:.1f}s frames={len(self.rows)} base_z={height:.3f}{progress}", flush=True)
            if self.route is not None and not self.route.done:
                raise RuntimeError("Square route timed out before completing all four sides")
        except BaseException as error:
            self.failure = str(error) or type(error).__name__
            raise
        finally:
            if viewer is not None: viewer.close()
            self.write_results(time.monotonic()-start)

    def write_results(self, wall_time):
        directory = self.args.output
        directory.mkdir(parents=True, exist_ok=True)
        self.input_file.close()
        for filename, rows in [("samples.csv", self.rows)]:
            if rows:
                with (directory / filename).open("w") as f:
                    writer = csv.DictWriter(f, fieldnames=list(rows[0]))
                    writer.writeheader(); writer.writerows(rows)
        completed = self.route.done if self.route is not None else len(self.rows) == getattr(self, "expected", -1)
        summary = {"completed": self.failure is None and completed,
                   "failure": self.failure, "scenario": self.args.scenario, "all_frames": len(self.rows),
                   "simulation_hz": 1/self.dt, "wall_time_s": wall_time,
                   "foot_sensor_input": False, "joint_torque_source": "MuJoCo qfrc_actuator",
                   "foot_force_threshold_N": self.args.foot_threshold,
                   "alignment": "initial SE(2) pose only; no scale fitting"}
        samples = [r for r in self.rows if r["t"] >= self.args.warmup - 1e-9]
        if len(samples) >= 2:
            values = lambda *keys: np.array([[r[k] for k in keys] for r in samples])
            est, truth = values("capo_x", "capo_y"), values("gt_x", "gt_y")
            angle = samples[0]["capo_yaw"] - samples[0]["gt_yaw"]
            transform = np.array([[math.cos(angle), -math.sin(angle)], [math.sin(angle), math.cos(angle)]])
            aligned = (truth-truth[0]) @ transform.T + est[0]
            error = np.linalg.norm(est-aligned, axis=1)
            yaw = values("capo_yaw", "gt_yaw")
            yaw_error = (yaw[:, 0]-yaw[0, 0])-(yaw[:, 1]-yaw[0, 1])
            yaw_error = (yaw_error+math.pi)%(2*math.pi)-math.pi
            path = np.linalg.norm(np.diff(truth, axis=0), axis=1).sum()
            summary.update(duration_s=samples[-1]["t"]-samples[0]["t"], samples=len(samples),
                           ground_truth_path_m=float(path), endpoint_error_m=float(error[-1]),
                           endpoint_error_percent_of_path=float(100*error[-1]/max(path, 1e-6)),
                           position_rmse_m=float(np.sqrt(np.mean(error**2))),
                           max_position_error_m=float(error.max()),
                           gt_endpoint_distance_from_start_m=float(np.linalg.norm(truth[-1]-truth[0])),
                           odom_endpoint_distance_from_start_m=float(np.linalg.norm(est[-1]-est[0])),
                           yaw_rmse_deg=float(np.degrees(np.sqrt(np.mean(yaw_error**2)))),
                           velocity_rmse_m_s=float(np.sqrt(np.mean(np.sum((values("capo_vx", "capo_vy")-values("gt_vx", "gt_vy"))**2, axis=1)))),
                           height_rmse_m=float(np.sqrt(np.mean((values("capo_z")-values("gt_z"))**2))),
                           mean_equivalent_fz_N=values(*["fz_"+l for l in LEGS]).mean(axis=0).tolist(),
                           contact_fraction=values(*["contact_"+l for l in LEGS]).mean(axis=0).tolist(),
                           waist_angle_range_rad=self.waist_ranges)
            if self.route is not None:
                route = self.route.describe()
                route["corner_odometry"] = []
                previous_corner = self.route.origin
                for corner in self.route.corners:
                    i = min(range(len(samples)), key=lambda j: abs(samples[j]["t"]-corner["time_s"]))
                    actual = np.array(corner["actual_world_xy"])
                    route["corner_odometry"].append({"side": corner["side"], "time_s": corner["time_s"],
                        "side_chord_m": float(np.linalg.norm(actual-previous_corner)),
                        "waypoint_error_m": float(np.linalg.norm(actual-corner["target_world_xy"])),
                        "odometry_error_m": float(error[i])})
                    previous_corner = actual
                summary["square"] = route
            self.plot(samples, est, aligned, error, yaw_error)
        (directory / "summary.json").write_text(json.dumps(summary, indent=2)+"\n")
        print(json.dumps(summary, indent=2), flush=True)

    def plot(self, samples, estimate, truth, error, yaw_error):
        import matplotlib
        matplotlib.use("Agg")
        import mpl_toolkits
        toolkit = Path(matplotlib.__file__).parent.parent / "mpl_toolkits"
        if toolkit.is_dir(): mpl_toolkits.__path__ = [str(toolkit)]
        import matplotlib.pyplot as plt
        fig, axes = plt.subplots(2, 2, figsize=(12, 8), constrained_layout=True)
        t = [r["t"] for r in samples]
        axes[0, 0].plot(truth[:, 0], truth[:, 1], label="MuJoCo (initial pose aligned)")
        axes[0, 0].plot(estimate[:, 0], estimate[:, 1], label="CAPO")
        if self.route is not None and self.route.waypoints is not None:
            angle = samples[0]["capo_yaw"]-samples[0]["gt_yaw"]
            transform = np.array([[math.cos(angle), -math.sin(angle)], [math.sin(angle), math.cos(angle)]])
            first_truth = np.array([samples[0]["gt_x"], samples[0]["gt_y"]])
            corners = (self.route.waypoints-first_truth) @ transform.T + estimate[0]
            axes[0, 0].plot(corners[:, 0], corners[:, 1], "k--", alpha=.5, label="Target square")
            axes[0, 0].scatter(*estimate[0], marker="o", color="green", label="Start")
            axes[0, 0].scatter(*estimate[-1], marker="x", color="red", label="CAPO finish")
        axes[0, 0].axis("equal"); axes[0, 0].legend()
        axes[0, 0].set(xlabel="x [m]", ylabel="y [m]", title="Marsdog "+self.args.scenario)
        axes[0, 1].plot(t, error); axes[0, 1].set(xlabel="Time [s]", ylabel="Position error [m]")
        for leg in LEGS: axes[1, 0].plot(t, [r["fz_"+leg] for r in samples], label=leg, alpha=0.7)
        axes[1, 0].axhline(self.args.foot_threshold, color="black", linestyle="--")
        axes[1, 0].legend(); axes[1, 0].set(xlabel="Time [s]", ylabel="Equivalent Fz [N]")
        axes[1, 1].plot(t, np.degrees(yaw_error))
        axes[1, 1].set(xlabel="Time [s]", ylabel="Relative yaw error [deg]")
        fig.savefig(self.args.output / "comparison.png", dpi=160)
        plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sdk", type=Path, required=True)
    parser.add_argument("--urdf", type=Path)
    parser.add_argument("--scenario", choices=["stand", "straight", "turn", "square"], required=True)
    for name, default in [("duration", 20.), ("warmup", 4.), ("vx", .25), ("wz", .3),
                          ("initial-yaw", .7), ("foot-threshold", -8.), ("side-length", 10.), ("square-timeout", 0.)]:
        parser.add_argument("--"+name, type=float, default=default)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--viewer", action="store_true")
    parser.add_argument("--realtime", action="store_true")
    args = parser.parse_args()
    rclpy.init(args=[])
    node = None
    try:
        node = MarsdogBridge(args)
        node.run()
    finally:
        if node is not None: node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
