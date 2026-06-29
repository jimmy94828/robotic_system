#!/usr/bin/env bash
set -euo pipefail

SESSION_NAME="${SESSION_NAME:-agent_mock_suite}"
OBJECT_QUERY_REPLY_INDEX="${OBJECT_QUERY_REPLY_INDEX:-0}"
OBJECT_QUERY_REPLY_RANDOM="${OBJECT_QUERY_REPLY_RANDOM:-0}"
TASK_SUITE_INITIAL_DELAY_SEC="${TASK_SUITE_INITIAL_DELAY_SEC:-15}"
TASK_SUITE_BETWEEN_TASKS_SEC="${TASK_SUITE_BETWEEN_TASKS_SEC:-3}"
TASK_SUITE_TASK_TIMEOUT_SEC="${TASK_SUITE_TASK_TIMEOUT_SEC:-300}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if ! command -v tmux >/dev/null 2>&1; then
  echo "tmux is required for this helper."
  exit 1
fi

if tmux has-session -t "${SESSION_NAME}" 2>/dev/null; then
  echo "tmux session '${SESSION_NAME}' already exists."
  echo "Attach with: tmux attach -t ${SESSION_NAME}"
  echo "Kill it with: tmux kill-session -t ${SESSION_NAME}"
  exit 1
fi

SESSION_NAME="${SESSION_NAME}" AUTO_PUBLISH=0 "${SCRIPT_DIR}/agent_mock_pipeline_tmux.sh"

SETUP_CMD="cd /robot_ws && source /opt/ros/humble/setup.bash && source /opt/conda/etc/profile.d/conda.sh && conda activate robot_ros && source install/setup.bash"

tmux new-window -t "${SESSION_NAME}" -n "suite"
REPLY_PANE="${SESSION_NAME}:suite.0"
PUBLISH_PANE="$(tmux split-window -h -t "${REPLY_PANE}" -P -F '#{pane_id}')"

tmux send-keys -t "${REPLY_PANE}" "set +H" C-m
tmux send-keys -t "${PUBLISH_PANE}" "set +H" C-m

tmux send-keys -t "${REPLY_PANE}" "${SETUP_CMD}" C-m
tmux send-keys -t "${REPLY_PANE}" "export OBJECT_QUERY_REPLY_INDEX=${OBJECT_QUERY_REPLY_INDEX} OBJECT_QUERY_REPLY_RANDOM=${OBJECT_QUERY_REPLY_RANDOM}; clear; echo '[auto_reply] starting'; python3 /robot_ws/scripts/auto_object_query_reply.py" C-m

tmux send-keys -t "${PUBLISH_PANE}" "${SETUP_CMD}" C-m
tmux send-keys -t "${PUBLISH_PANE}" "export TASK_SUITE_INITIAL_DELAY_SEC=${TASK_SUITE_INITIAL_DELAY_SEC} TASK_SUITE_BETWEEN_TASKS_SEC=${TASK_SUITE_BETWEEN_TASKS_SEC} TASK_SUITE_TASK_TIMEOUT_SEC=${TASK_SUITE_TASK_TIMEOUT_SEC}; clear; echo '[task_suite] starting'; python3 /robot_ws/scripts/publish_agent_task_suite.py" C-m

tmux select-window -t "${SESSION_NAME}:suite"

echo "Started mock task suite in tmux session '${SESSION_NAME}'."
echo "Attach with:"
echo "  tmux attach -t ${SESSION_NAME}"
echo
echo "Window 1: agent-mock pipeline"
echo "Window 2: suite"
echo "  left:  auto /object_query_reply"
echo "  right: task publisher waiting for /task_reply"
echo
echo "Object-query reply mode:"
echo "  OBJECT_QUERY_REPLY_INDEX=${OBJECT_QUERY_REPLY_INDEX}"
echo "  OBJECT_QUERY_REPLY_RANDOM=${OBJECT_QUERY_REPLY_RANDOM}"
