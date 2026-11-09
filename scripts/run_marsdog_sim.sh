#!/usr/bin/env bash
set -e
CAPO_REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CAPO_MARSDOG_WS="${CAPO_MARSDOG_WS:-$HOME/capo_marsdog_ws}"
CAPO_SIM_VENV="${CAPO_SIM_VENV:-$HOME/capo_sim_venv}"
source /opt/ros/humble/setup.bash
source "$CAPO_MARSDOG_WS/install/setup.bash"
export PYTHONDONTWRITEBYTECODE=1
exec "$CAPO_SIM_VENV/bin/python" -B "$CAPO_REPO_DIR/integration/run_marsdog_sim.py" "$@"
