#!/usr/bin/env bash
set -euo pipefail

SESSION_NAME="${SESSION_NAME:-agent_mock}"
COMMAND_TEXT="${1:-bring pringles on table to sofa}"
AUTO_PUBLISH="${AUTO_PUBLISH:-0}"
PUBLISH_DELAY_SEC="${PUBLISH_DELAY_SEC:-8}"
PLANNER_BACKEND="${PLANNER_BACKEND:-vllm}"
VLLM_BASE_URL="${VLLM_BASE_URL:-http://localhost:8001}"
VLLM_MODEL="${VLLM_MODEL:-qwen-vl}"
QWEN_MAX_NEW_TOKENS="${QWEN_MAX_NEW_TOKENS:-256}"
VLLM_TEMPERATURE="${VLLM_TEMPERATURE:-0}"
VLLM_TIMEOUT_SEC="${VLLM_TIMEOUT_SEC:-60}"
ENABLE_PRE_ACTION_VIEW_CHECK="${ENABLE_PRE_ACTION_VIEW_CHECK:-true}"
VIEW_CHECK_MAX_ADJUSTMENTS="${VIEW_CHECK_MAX_ADJUSTMENTS:-3}"
RECOVER_GRASP_FAILURE_WITH_REALIGN="${RECOVER_GRASP_FAILURE_WITH_REALIGN:-false}"
VIEW_CRITIC_BACKEND="${VIEW_CRITIC_BACKEND:-mock}"
VIEW_CRITIC_VLLM_BASE_URL="${VIEW_CRITIC_VLLM_BASE_URL:-http://localhost:8001/v1}"
VIEW_CRITIC_VLLM_MODEL="${VIEW_CRITIC_VLLM_MODEL:-qwen-vl}"
VIEW_CRITIC_IMAGE_TOPIC="${VIEW_CRITIC_IMAGE_TOPIC:-/camera/color/image_raw}"
VIEW_ADJUST_MAX_YAW_DEG="${VIEW_ADJUST_MAX_YAW_DEG:-240.0}"
VIEW_ADJUST_MAX_FORWARD_M="${VIEW_ADJUST_MAX_FORWARD_M:-0.25}"

if ! command -v tmux >/dev/null 2>&1; then
  echo "tmux is required for this helper. Install tmux or run the three commands manually."
  exit 1
fi

TMUX_CONF="${HOME}/.tmux.conf"
mkdir -p "$(dirname "${TMUX_CONF}")"
if ! grep -q '^# >>> robotic-agent tmux copy settings' "${TMUX_CONF}" 2>/dev/null; then
  {
    echo '# >>> robotic-agent tmux copy settings'
    echo 'set -g mouse on'
    echo 'set -g history-limit 50000'
    echo 'setw -g mode-keys vi'
    echo 'set -g set-clipboard on'
    echo 'bind-key -T copy-mode-vi v send-keys -X begin-selection'
    echo 'bind-key -T copy-mode-vi y send-keys -X copy-selection-and-cancel'
    echo 'bind-key -T copy-mode-vi MouseDragEnd1Pane send-keys -X copy-selection-and-cancel'
    echo '# <<< robotic-agent tmux copy settings'
  } >> "${TMUX_CONF}"
fi

if tmux has-session -t "${SESSION_NAME}" 2>/dev/null; then
  echo "tmux session '${SESSION_NAME}' already exists."
  echo "Attach with: tmux attach -t ${SESSION_NAME}"
  echo "Kill it with: tmux kill-session -t ${SESSION_NAME}"
  exit 1
fi

SETUP_CMD="cd /robot_ws && source /opt/ros/humble/setup.bash && source /opt/conda/etc/profile.d/conda.sh && conda activate robot_ros && source install/setup.bash && export LLM_BACKEND=${PLANNER_BACKEND} VLLM_BASE_URL=${VLLM_BASE_URL} VLLM_MODEL=${VLLM_MODEL} QWEN_MAX_NEW_TOKENS=${QWEN_MAX_NEW_TOKENS} VLLM_TEMPERATURE=${VLLM_TEMPERATURE} VLLM_TIMEOUT_SEC=${VLLM_TIMEOUT_SEC} ENABLE_PRE_ACTION_VIEW_CHECK=${ENABLE_PRE_ACTION_VIEW_CHECK} VIEW_CHECK_MAX_ADJUSTMENTS=${VIEW_CHECK_MAX_ADJUSTMENTS} VIEW_CRITIC_BACKEND=${VIEW_CRITIC_BACKEND} VIEW_CRITIC_VLLM_BASE_URL=${VIEW_CRITIC_VLLM_BASE_URL} VIEW_CRITIC_VLLM_MODEL=${VIEW_CRITIC_VLLM_MODEL} VIEW_CRITIC_IMAGE_TOPIC=${VIEW_CRITIC_IMAGE_TOPIC} VIEW_ADJUST_MAX_FORWARD_M=${VIEW_ADJUST_MAX_FORWARD_M} RECOVER_GRASP_FAILURE_WITH_REALIGN=${RECOVER_GRASP_FAILURE_WITH_REALIGN} && AGENT_EXE=\$(ros2 pkg prefix decision_maker)/lib/decision_maker/agent_decision_maker_node && if [ -f \"\$AGENT_EXE\" ]; then sed -i \"1s|.*|#!/opt/conda/envs/robot_ros/bin/python3|\" \"\$AGENT_EXE\"; fi"

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
tmux send-keys -t "${COMMAND_PANE}" "clear; echo \"[manual_command] ready with LLM_BACKEND=\$LLM_BACKEND VLLM_BASE_URL=\$VLLM_BASE_URL VLLM_MODEL=\$VLLM_MODEL max_tokens=\$QWEN_MAX_NEW_TOKENS\"" C-m
if [ "${AUTO_PUBLISH}" = "1" ]; then
  tmux send-keys -t "${COMMAND_PANE}" "sleep ${PUBLISH_DELAY_SEC}; ros2 topic pub --once /manual_command std_msgs/msg/String \"{data: '${COMMAND_TEXT}'}\"" C-m
fi

# Bottom full-width: decision node.
tmux send-keys -t "${AGENT_PANE}" "${SETUP_CMD}" C-m
tmux send-keys -t "${AGENT_PANE}" "clear; echo \"[agent] starting with LLM_BACKEND=\$LLM_BACKEND VLLM_BASE_URL=\$VLLM_BASE_URL VLLM_MODEL=\$VLLM_MODEL max_tokens=\$QWEN_MAX_NEW_TOKENS\"; ros2 launch decision_maker agent_mock_pipeline.launch.py" C-m

tmux select-pane -t "${COMMAND_PANE}"

echo "Started tmux session '${SESSION_NAME}'."
echo "Attach with:"
echo "  tmux attach -t ${SESSION_NAME}"
echo
echo "Object query pane: top-left  ros2 run object_query object_query_server"
echo "Agent pane:        bottom    ros2 launch decision_maker agent_mock_pipeline.launch.py"
echo "Command pane:      top-right interactive shell"
echo "Planner backend:   ${PLANNER_BACKEND}"
if [ "${PLANNER_BACKEND}" = "vllm" ]; then
  echo "vLLM base URL:    ${VLLM_BASE_URL}"
  echo "vLLM model:       ${VLLM_MODEL}"
  echo "Max new tokens:   ${QWEN_MAX_NEW_TOKENS}"
  echo "Temperature:      ${VLLM_TEMPERATURE}"
fi
echo "View check:       ${ENABLE_PRE_ACTION_VIEW_CHECK}"
echo "View backend:     ${VIEW_CRITIC_BACKEND}"
echo "View max yaw:     ${VIEW_ADJUST_MAX_YAW_DEG} deg"
echo "Grasp recovery:   ${RECOVER_GRASP_FAILURE_WITH_REALIGN}"
echo "View vLLM URL:    ${VIEW_CRITIC_VLLM_BASE_URL}"
echo "View vLLM model:  ${VIEW_CRITIC_VLLM_MODEL}"
echo "View image topic: ${VIEW_CRITIC_IMAGE_TOPIC}"
echo "View max adjust:  ${VIEW_CHECK_MAX_ADJUSTMENTS}"
echo "View max forward: ${VIEW_ADJUST_MAX_FORWARD_M} m"
echo "Mouse scroll:      enabled in ${TMUX_CONF}"
if [ "${AUTO_PUBLISH}" = "1" ]; then
  echo "Publisher pane:    will publish after ${PUBLISH_DELAY_SEC}s:"
  echo "  ${COMMAND_TEXT}"
else
  echo "Publisher pane:    interactive shell only."
fi
