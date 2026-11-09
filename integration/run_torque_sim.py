"""Run an isolated CAPO estimator together with the torque-only MuJoCo bridge."""
import argparse
from datetime import datetime
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import yaml
from zoneinfo import ZoneInfo

REPO = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenario", choices=["stand", "straight", "turn", "curve", "all"], default="straight")
    parser.add_argument("--duration", type=float, default=20.)
    parser.add_argument("--warmup", type=float, default=6.)
    parser.add_argument("--vx", type=float, default=0.4)
    parser.add_argument("--wz", type=float, default=0.4)
    parser.add_argument("--initial-yaw", type=float, default=0.7)
    parser.add_argument("--foot-threshold", type=float, default=-20.)
    parser.add_argument("--viewer", action="store_true")
    parser.add_argument("--realtime", action="store_true")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    output = args.output or REPO / ".runtime/results" / datetime.now(ZoneInfo("Asia/Shanghai")).strftime("%Y%m%d-%H%M%S")
    output = output.resolve()
    scenarios = ["stand", "straight", "turn"] if args.scenario == "all" else [args.scenario]
    environment = os.environ.copy()
    environment["ROS_DOMAIN_ID"] = environment.get("CAPO_SIM_DOMAIN_ID", "71")
    environment["ROS_LOCALHOST_ONLY"] = "1"
    for scenario in scenarios:
        directory = output / scenario
        directory.mkdir(parents=True, exist_ok=True)
        environment["ROS_LOG_DIR"] = str(directory / "ros_logs")
        config = yaml.safe_load((REPO / "config/sim_torque.yaml").read_text())
        config["fusion_estimator_node"]["ros__parameters"]["foot_force_threshold"] = args.foot_threshold
        config_path = directory / "config.yaml"
        config_path.write_text(yaml.safe_dump(config))
        with (directory / "capo.log").open("w") as log:
            estimator = subprocess.Popen([
                "ros2", "launch", "fusion_estimator", "go2_capo_torque_sim.launch.py",
                f"config:={config_path}",
            ], env=environment, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            try:
                command = [sys.executable, str(REPO / "integration/capo_ros2_bridge.py"),
                           "--scenario", scenario, "--duration", str(args.duration),
                           "--warmup", str(args.warmup), "--vx", str(args.vx), "--wz", str(args.wz),
                           "--initial-yaw", str(args.initial_yaw),
                           "--foot-threshold", str(args.foot_threshold), "--output", str(directory)]
                if args.viewer:
                    command.append("--viewer")
                if args.realtime:
                    command.append("--realtime")
                subprocess.run(command, env=environment, check=True)
            finally:
                if estimator.poll() is None:
                    os.killpg(estimator.pid, signal.SIGINT)
                    try:
                        estimator.wait(timeout=10)
                    except subprocess.TimeoutExpired:
                        os.killpg(estimator.pid, signal.SIGTERM)
                        estimator.wait(timeout=5)
                if estimator.returncode not in (0, -signal.SIGINT):
                    print(f"Estimator exited with {estimator.returncode}; inspect {directory / 'capo.log'}")
        print(f"Results: {directory}", flush=True)
        # Allow discovery to remove the previous estimator before resetting /clock.
        if len(scenarios) > 1:
            time.sleep(1)


if __name__ == "__main__":
    main()
