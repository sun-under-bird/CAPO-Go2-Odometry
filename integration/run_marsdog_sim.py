"""Run the Marsdog simulation adaptation and independent ground-truth evaluation."""
import argparse
from datetime import datetime
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
from zoneinfo import ZoneInfo

import mujoco
import yaml
from marsdog_model import DEFAULT_SDK, estimator_geometry, load_sdk, provenance
from marsdog_urdf import DEFAULT_URDF, inspect_urdf

REPO = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sdk", type=Path, help="Policy/runtime root (previous SDK when absent from measured source)")
    model_source = parser.add_mutually_exclusive_group()
    model_source.add_argument("--urdf", type=Path, default=DEFAULT_URDF,
                              help="Authoritative robot geometry and inertial data")
    model_source.add_argument("--legacy-sdk-model", action="store_true", help="Reproduce the old training-model baseline")
    parser.add_argument("--scenario", choices=["stand", "straight", "turn", "square", "all"], default="straight")
    parser.add_argument("--duration", type=float, default=20.)
    parser.add_argument("--warmup", type=float, default=4.)
    parser.add_argument("--vx", type=float, default=0.25)
    parser.add_argument("--wz", type=float, default=0.3)
    parser.add_argument("--initial-yaw", type=float, default=0.7)
    parser.add_argument("--foot-threshold", type=float, default=-8.)
    parser.add_argument("--estimator-profile", choices=["baseline", "angular", "motion", "weighted", "optimized"], default="optimized",
                        help="Ablation: baseline, body angular term, plus rolling, plus contact confidence, plus timestamp prediction")
    parser.add_argument("--side-length", type=float, default=10., help="Square side length in metres")
    parser.add_argument("--square-timeout", type=float, default=0., help="Square time limit; 0 selects an automatic limit")
    parser.add_argument("--viewer", action="store_true")
    parser.add_argument("--realtime", action="store_true")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    numbers = [args.duration, args.warmup, args.vx, args.wz, args.initial_yaw, args.foot_threshold, args.side_length, args.square_timeout]
    if (not all(map(math.isfinite, numbers)) or args.duration <= 0 or args.warmup < 3 or
            not -0.3 <= args.vx <= 0.4 or abs(args.wz) > 0.6 or args.foot_threshold >= 0):
        parser.error("Require finite values, duration>0, warmup>=3, vx in [-0.3,0.4], |wz|<=0.6, threshold<0")
    if args.side_length <= 0 or args.square_timeout < 0 or (args.scenario == "square" and args.vx < .15):
        parser.error("Square requires side-length>0, square-timeout>=0 and vx>=0.15")
    args.urdf = None if args.legacy_sdk_model else args.urdf.resolve()
    local_sdk = args.urdf.parent if args.urdf else DEFAULT_SDK
    if not (local_sdk / "sim/policy/policy.onnx").is_file():
        local_sdk = DEFAULT_SDK
    args.sdk = (args.sdk or local_sdk).resolve()
    _, _, model, names, _, _, _ = load_sdk(args.sdk, args.urdf)
    geometry = estimator_geometry(model, names)
    output = (args.output or REPO / ".runtime/marsdog" /
              datetime.now(ZoneInfo("Asia/Shanghai")).strftime("%Y%m%d-%H%M%S-%f")).resolve()
    scenarios = ["stand", "straight", "turn"] if args.scenario == "all" else [args.scenario]
    for scenario in scenarios:
        directory = output / scenario
        directory.mkdir(parents=True, exist_ok=False)
        config = yaml.safe_load((REPO / "config/marsdog_sim.yaml").read_text())
        config["fusion_estimator_node"]["ros__parameters"].update(geometry)
        config["fusion_estimator_node"]["ros__parameters"]["foot_force_threshold"] = args.foot_threshold
        config["fusion_estimator_node"]["ros__parameters"].update(
            point_angular_compensation=args.estimator_profile != "baseline",
            point_rolling_compensation=args.estimator_profile in ("motion", "weighted", "optimized"),
            point_contact_weighting=args.estimator_profile in ("weighted", "optimized"),
            point_timestamp_prediction=args.estimator_profile == "optimized")
        config_path = directory / "config.yaml"
        config_path.write_text(yaml.safe_dump(config, sort_keys=False))
        mujoco.mj_saveModel(model, str(directory / "model.mjb"))
        manifest = {"sdk": str(args.sdk), "sha256": provenance(args.sdk),
                    "mujoco": mujoco.__version__, "physics_dt": float(model.opt.timestep),
                    "model_source": "measured URDF" if args.urdf else "delivered 2100 simulation MJCF with runtime configuration",
                    "urdf": inspect_urdf(args.urdf) if args.urdf else None,
                    "compiled_model_sha256": hashlib.sha256((directory / "model.mjb").read_bytes()).hexdigest(),
                    "estimator_profile": args.estimator_profile,
                    "estimator_source_sha256": {str(p.relative_to(REPO)): hashlib.sha256(p.read_bytes()).hexdigest()
                                                 for p in sorted((REPO / "FusionEstimator").rglob("*"))
                                                 if p.suffix in (".h", ".cpp", ".c")},
                    "simulated_assumptions": ["URDF/model IMU extrinsic", "ideal calibrated actuator torque",
                                              "SDK controller PD, effort limits and motor armature",
                                              "flat ground friction=1; robot friction=0.6; robot self-contact disabled",
                                              "noise-free timestamped sensors", "flat-ground spherical contact geometry"],
                    "ground_truth_used_by_estimator": False,
                    "contact_truth_used_by_estimator": False,
                    "route_controller_uses_sim_position": scenario == "square",
                    "square_side_length_m": args.side_length if scenario == "square" else None}
        (directory / "provenance.json").write_text(json.dumps(manifest, indent=2) + "\n")
        env = os.environ.copy()
        env.update(ROS_DOMAIN_ID=env.get("CAPO_MARSDOG_DOMAIN_ID", "72"), ROS_LOCALHOST_ONLY="1",
                   ROS_LOG_DIR=str(directory / "ros_logs"), CAPO_PARAMS_FILE=str(config_path))
        with (directory / "capo.log").open("w") as log:
            estimator = subprocess.Popen(["ros2", "run", "fusion_estimator", "fusion_estimator_node"],
                                         env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            try:
                command = [sys.executable, "-B", str(REPO / "integration/marsdog_ros2_bridge.py"),
                           "--sdk", str(args.sdk), "--scenario", scenario, "--duration", str(args.duration),
                           "--warmup", str(args.warmup), "--vx", str(args.vx), "--wz", str(args.wz),
                           "--initial-yaw", str(args.initial_yaw), "--foot-threshold", str(args.foot_threshold),
                           "--side-length", str(args.side_length), "--square-timeout", str(args.square_timeout),
                           "--output", str(directory)]
                if args.urdf is not None: command += ["--urdf", str(args.urdf)]
                if args.viewer: command.append("--viewer")
                if args.realtime: command.append("--realtime")
                subprocess.run(command, env=env, check=True)
            finally:
                if estimator.poll() is None:
                    os.killpg(estimator.pid, signal.SIGINT)
                    try:
                        estimator.wait(timeout=10)
                    except subprocess.TimeoutExpired:
                        os.killpg(estimator.pid, signal.SIGTERM)
                        estimator.wait(timeout=5)
        print(f"Results: {directory}", flush=True)
        if len(scenarios) > 1: time.sleep(1)


if __name__ == "__main__":
    main()
