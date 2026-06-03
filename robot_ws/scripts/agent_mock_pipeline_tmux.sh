#!/usr/bin/env bash
set -euo pipefail

SESSION_NAME="${SESSION_NAME:-agent_mock}"
COMMAND_TEXT="${1:-bring pringles on table to sofa}"
AUTO_PUBLISH="${AUTO_PUBLISH:-0}"
PUBLISH_DELAY_SEC="${PUBLISH_DELAY_SEC:-8}"
PLANNER_BACKEND="${PLANNER_BACKEND:-local_transformers}"

if ! command -v tmux >/dev/null 2>&1; then
  echo "tmux is required for this helper. Install tmux or run the three commands manually."
  exit 1
fi

TMUX_CONF="${HOME}/.tmux.conf"
if ! grep -q '^set -g mouse on' "${TMUX_CONF}" 2>/dev/null; then
  {
    echo 'set -g mouse on'
    echo 'set -g history-limit 50000'
  } >> "${TMUX_CONF}"
fi

if tmux has-session -t "${SESSION_NAME}" 2>/dev/null; then
  echo "tmux session '${SESSION_NAME}' already exists."
  echo "Attach with: tmux attach -t ${SESSION_NAME}"
  echo "Kill it with: tmux kill-session -t ${SESSION_NAME}"
  exit 1
fi

SETUP_CMD="cd /robot_ws && source /opt/ros/humble/setup.bash && source /opt/conda/etc/profile.d/conda.sh && conda activate robot_ros && source install/setup.bash && export LLM_BACKEND=${PLANNER_BACKEND} && AGENT_EXE=\$(ros2 pkg prefix decision_maker)/lib/decision_maker/agent_decision_maker_node && if [ -f \"\$AGENT_EXE\" ]; then sed -i \"1s|.*|#!/opt/conda/envs/robot_ros/bin/python3|\" \"\$AGENT_EXE\"; fi"

# Layout:
#   top-left:  object_query
#   top-right: manual command shell
#   bottom:    decision node, full width
OBJECT_PANE=$(tmux new-session -d -s "${SESSION_NAME}" -n "agent-mock" -P -F '#{pane_id}' bash)
AGENT_PANE=$(tmux split-window -v -t "${OBJECT_PANE}" -P -F '#{pane_id}' bash)
COMMAND_PANE=$(tmux split-window -h -t "${OBJECT_PANE}" -P -F '#{pane_id}' bash)
tmux resize-pane -t "${AGENT_PANE}" -y 60%

for pane in "${OBJECT_PANE}" "${COMMAND_PANE}" "${AGENT_PANE}"; do
  tmux send-keys -t "${pane}" "set +H" C-m
done

# Top-left: object_query.
tmux send-keys -t "${OBJECT_PANE}" "${SETUP_CMD}" C-m
tmux send-keys -t "${OBJECT_PANE}" "clear; echo '[object_query] starting'; ros2 run object_query object_query_server" C-m

# Top-right: manual command shell.
tmux send-keys -t "${COMMAND_PANE}" "${SETUP_CMD}" C-m
tmux send-keys -t "${COMMAND_PANE}" "clear; echo \"[manual_command] ready with LLM_BACKEND=\$LLM_BACKEND\"" C-m
if [ "${AUTO_PUBLISH}" = "1" ]; then
  tmux send-keys -t "${COMMAND_PANE}" "sleep ${PUBLISH_DELAY_SEC}; ros2 topic pub --once /manual_command std_msgs/msg/String \"{data: '${COMMAND_TEXT}'}\"" C-m
fi

# Bottom full-width: decision node.
tmux send-keys -t "${AGENT_PANE}" "${SETUP_CMD}" C-m
tmux send-keys -t "${AGENT_PANE}" "clear; echo \"[agent] starting with LLM_BACKEND=\$LLM_BACKEND\"; ros2 launch decision_maker agent_mock_pipeline.launch.py" C-m

tmux select-pane -t "${COMMAND_PANE}"

echo "Started tmux session '${SESSION_NAME}'."
echo "Attach with:"
echo "  tmux attach -t ${SESSION_NAME}"
echo
echo "Object query pane: top-left  ros2 run object_query object_query_server"
echo "Agent pane:        bottom    ros2 launch decision_maker agent_mock_pipeline.launch.py"
echo "Command pane:      top-right interactive shell"
echo "Planner backend:   ${PLANNER_BACKEND}"
echo "Mouse scroll:      enabled in ${TMUX_CONF}"
if [ "${AUTO_PUBLISH}" = "1" ]; then
  echo "Publisher pane:    will publish after ${PUBLISH_DELAY_SEC}s:"
  echo "  ${COMMAND_TEXT}"
else
  echo "Publisher pane:    interactive shell only."
fi
