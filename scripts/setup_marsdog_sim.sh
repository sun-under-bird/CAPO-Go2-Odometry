#!/usr/bin/env bash
set -e
CAPO_REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CAPO_MARSDOG_WS="${CAPO_MARSDOG_WS:-$HOME/capo_marsdog_ws}"
source /opt/ros/humble/setup.bash
colcon --log-base "$CAPO_MARSDOG_WS/log" build \
  --base-paths "$CAPO_REPO_DIR" \
  --build-base "$CAPO_MARSDOG_WS/build" \
  --install-base "$CAPO_MARSDOG_WS/install" \
  --packages-select fusion_estimator \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DCAPO_BUILD_UNITREE_NODES=OFF
