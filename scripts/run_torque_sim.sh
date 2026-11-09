#!/usr/bin/env bash
set -e
CAPO_REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CAPO_SIM_WS="${CAPO_SIM_WS:-$HOME/capo_torque_ws}"
CAPO_SIM_VENV="${CAPO_SIM_VENV:-$HOME/capo_sim_venv}"
source /opt/ros/humble/setup.bash
source "$CAPO_SIM_WS/install/setup.bash"
exec "$CAPO_SIM_VENV/bin/python" "$CAPO_REPO_DIR/integration/run_torque_sim.py" "$@"
