#!/usr/bin/env bash
set -e
CAPO_REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CAPO_SIM_WS="${CAPO_SIM_WS:-$HOME/capo_torque_ws}"
CAPO_SIM_VENV="${CAPO_SIM_VENV:-$HOME/capo_sim_venv}"
source /opt/ros/humble/setup.bash
if [ ! -x "$CAPO_SIM_VENV/bin/pip" ]; then
  if ! python3 -c 'import virtualenv' 2>/dev/null; then
    python3 -m pip install --user 'virtualenv==20.35.4'
  fi
  python3 -m virtualenv --system-site-packages "$CAPO_SIM_VENV"
fi
"$CAPO_SIM_VENV/bin/python" -m pip install -r "$CAPO_REPO_DIR/integration/requirements-torque-sim.txt"
python3 "$CAPO_REPO_DIR/integration/fetch_go2_model.py" "$CAPO_REPO_DIR/.runtime/go2"
colcon --log-base "$CAPO_SIM_WS/log" build \
  --base-paths "$CAPO_REPO_DIR" \
  --build-base "$CAPO_SIM_WS/build" \
  --install-base "$CAPO_SIM_WS/install" \
  --packages-select fusion_estimator \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DCAPO_BUILD_UNITREE_NODES=OFF
