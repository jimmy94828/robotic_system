#!/usr/bin/env bash
# Launch the RViz2 visualization stack in a tmux session.
# Run this script from the HOST machine (outside Docker).
#
# Usage:
#   ./scripts/viz_tmux.sh                         # no fixed reference markers
#   ./scripts/viz_tmux.sh table:0 chair:0 sofa:0  # with reference markers
#
# Layout:
#   left  : show_selected_instance_markers.py  (marker publisher + point cloud)
#   right : rviz2
set -euo pipefail

CONTAINER="${CONTAINER:-robotic_agent_system}"
SESSION_NAME="${SESSION_NAME:-viz}"
SELECTIONS="${*:-}"

# ---------------------------------------------------------------------------
# Pre-flight checks
# ---------------------------------------------------------------------------
if ! command -v tmux >/dev/null 2>&1; then
  echo "ERROR: tmux is required. Install it or run the two commands manually."
  exit 1
fi

if ! docker ps --format '{{.Names}}' | grep -q "^${CONTAINER}$"; then
  echo "ERROR: Container '${CONTAINER}' is not running."
  echo "  Start it: cd ~/robotic && docker compose up -d"
  exit 1
fi

if tmux has-session -t "${SESSION_NAME}" 2>/dev/null; then
  echo "tmux session '${SESSION_NAME}' already exists."
  echo "  attach : tmux attach -t ${SESSION_NAME}"
  echo "  kill   : tmux kill-session -t ${SESSION_NAME}"
  exit 1
fi

# Allow the container to open GUI windows on the host display
xhost +local:root 2>/dev/null || true

# ---------------------------------------------------------------------------
# Persistent tmux tweaks (mouse scroll + large history)
# ---------------------------------------------------------------------------
TMUX_CONF="${HOME}/.tmux.conf"
if ! grep -q '^set -g mouse on' "${TMUX_CONF}" 2>/dev/null; then
  {
    echo 'set -g mouse on'
    echo 'set -g history-limit 50000'
  } >> "${TMUX_CONF}"
fi

# ---------------------------------------------------------------------------
# ROS environment setup (runs inside the container)
# ---------------------------------------------------------------------------
SETUP_CMD="cd /robot_ws \
  && source /opt/ros/humble/setup.bash \
  && source /opt/conda/etc/profile.d/conda.sh \
  && conda activate robot_ros \
  && source install/setup.bash"

# ---------------------------------------------------------------------------
# Create session and split into two side-by-side panes
# ---------------------------------------------------------------------------
MARKER_PANE=$(tmux new-session -d -s "${SESSION_NAME}" -n "viz" -P -F '#{pane_id}' bash)
RVIZ_PANE=$(tmux split-window -h -t "${MARKER_PANE}" -P -F '#{pane_id}' bash)

for pane in "${MARKER_PANE}" "${RVIZ_PANE}"; do
  tmux send-keys -t "${pane}" "set +H" C-m
  # Enter the container
  tmux send-keys -t "${pane}" "docker exec -it ${CONTAINER} bash" C-m
  sleep 0.3
done

# ---------------------------------------------------------------------------
# Left pane: marker publisher
# ---------------------------------------------------------------------------
tmux send-keys -t "${MARKER_PANE}" "${SETUP_CMD}" C-m
tmux send-keys -t "${MARKER_PANE}" \
  "clear; echo '[viz] starting marker publisher'; python3 tools/show_selected_instance_markers.py ${SELECTIONS}" C-m

# ---------------------------------------------------------------------------
# Right pane: RViz2
# (small delay so the marker publisher can advertise its topics first)
# ---------------------------------------------------------------------------
tmux send-keys -t "${RVIZ_PANE}" "${SETUP_CMD}" C-m
tmux send-keys -t "${RVIZ_PANE}" \
  "clear; echo '[viz] waiting for marker publisher...'; sleep 3 && rviz2" C-m

# Focus on the RViz pane so the user sees it first when attaching
tmux select-pane -t "${RVIZ_PANE}"

# ---------------------------------------------------------------------------
# Done
# ---------------------------------------------------------------------------
echo "Started tmux session '${SESSION_NAME}'."
echo ""
echo "  attach : tmux attach -t ${SESSION_NAME}"
echo ""
echo "  left pane  : show_selected_instance_markers.py ${SELECTIONS}"
echo "  right pane : rviz2  (container: ${CONTAINER})"
echo ""
echo "In RViz2 add these displays:"
echo "  PointCloud2  -> /aligned_map_pointcloud_preview"
echo "  MarkerArray  -> /semantic_map_markers_preview"
echo "  Path         -> /semantic_map_preview_path"
echo "  Global Options -> Fixed Frame: map"
