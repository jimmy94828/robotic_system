#!/usr/bin/env bash
set -euo pipefail

SESSION_NAME="${SESSION_NAME:-agent_live}"
PLANNER_BACKEND="${PLANNER_BACKEND:-vllm}"
VLLM_BASE_URL="${VLLM_BASE_URL:-http://localhost:8001}"
VLLM_MODEL="${VLLM_MODEL:-decision-qwen}"
QWEN_MAX_NEW_TOKENS="${QWEN_MAX_NEW_TOKENS:-256}"
VLLM_TEMPERATURE="${VLLM_TEMPERATURE:-0}"
VLLM_TIMEOUT_SEC="${VLLM_TIMEOUT_SEC:-60}"
KACHAKA_ENDPOINT="${KACHAKA_ENDPOINT:-192.168.1.3:26400}"
AGENT_MAX_REPLANS="${AGENT_MAX_REPLANS:-2}"

if ! command -v tmux >/dev/null 2>&1; then
  echo "tmux is required for this helper. Install tmux or run the four commands manually."
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

SETUP_CMD="cd /robot_ws && source /opt/ros/humble/setup.bash && source /opt/conda/etc/profile.d/conda.sh && conda activate robot_ros && source install/setup.bash && export LLM_BACKEND=${PLANNER_BACKEND} VLLM_BASE_URL=${VLLM_BASE_URL} VLLM_MODEL=${VLLM_MODEL} QWEN_MAX_NEW_TOKENS=${QWEN_MAX_NEW_TOKENS} VLLM_TEMPERATURE=${VLLM_TEMPERATURE} VLLM_TIMEOUT_SEC=${VLLM_TIMEOUT_SEC} && AGENT_EXE=\$(ros2 pkg prefix decision_maker)/lib/decision_maker/agent_decision_maker_node && if [ -f \"\$AGENT_EXE\" ]; then sed -i \"1s|.*|#!/opt/conda/envs/robot_ros/bin/python3|\" \"\$AGENT_EXE\"; fi"

# Layout:
#   top-left:     object_query server
#   top-right:    Kachaka navigation server
#   bottom-left:  agent decision node
#   bottom-right: natural-language command input
OBJECT_PANE=$(tmux new-session -d -s "${SESSION_NAME}" -n "agent-live" -P -F '#{pane_id}' bash)
BOTTOM_LEFT_PANE=$(tmux split-window -v -t "${OBJECT_PANE}" -P -F '#{pane_id}' bash)
NAV_PANE=$(tmux split-window -h -t "${OBJECT_PANE}" -P -F '#{pane_id}' bash)
NL_PANE=$(tmux split-window -h -t "${BOTTOM_LEFT_PANE}" -P -F '#{pane_id}' bash)
tmux resize-pane -t "${BOTTOM_LEFT_PANE}" -y 55%

for pane in "${OBJECT_PANE}" "${NAV_PANE}" "${BOTTOM_LEFT_PANE}" "${NL_PANE}"; do
  tmux send-keys -t "${pane}" "set +H" C-m
done

# Top-left: object_query server.
tmux send-keys -t "${OBJECT_PANE}" "${SETUP_CMD}" C-m
tmux send-keys -t "${OBJECT_PANE}" "clear; echo '[object_query] starting'; ros2 run object_query object_query_server" C-m

# Top-right: Kachaka navigation server.
tmux send-keys -t "${NAV_PANE}" "${SETUP_CMD}" C-m
tmux send-keys -t "${NAV_PANE}" "clear; echo '[kachaka_nav] starting with KACHAKA_ENDPOINT=${KACHAKA_ENDPOINT}'; export PYTHONPATH=\$PYTHONPATH:/opt/conda/envs/robot_ros/lib/python3.10/site-packages; ros2 run kachaka_nav modular_nav_node --ros-args -p kachaka_ip:=${KACHAKA_ENDPOINT}" C-m

# Bottom-left: live agent decision node.
tmux send-keys -t "${BOTTOM_LEFT_PANE}" "${SETUP_CMD}" C-m
tmux send-keys -t "${BOTTOM_LEFT_PANE}" "clear; echo \"[agent] starting live pipeline with LLM_BACKEND=\$LLM_BACKEND VLLM_BASE_URL=\$VLLM_BASE_URL VLLM_MODEL=\$VLLM_MODEL max_tokens=\$QWEN_MAX_NEW_TOKENS, replans=${AGENT_MAX_REPLANS}\"; ros2 run decision_maker agent_decision_maker_node --ros-args -p enable_map_visualizer:=false -p mock_execution:=false -p agent_max_replans:=${AGENT_MAX_REPLANS}" C-m

# Bottom-right: natural-language command input node.
tmux send-keys -t "${NL_PANE}" "${SETUP_CMD}" C-m
tmux send-keys -t "${NL_PANE}" "clear; echo '[nl_command] starting'; ros2 run decision_maker nl_command_node" C-m

tmux select-pane -t "${NL_PANE}"

echo "Started tmux session '${SESSION_NAME}'."
echo "Attach with:"
echo "  tmux attach -t ${SESSION_NAME}"
echo
echo "Object query pane: top-left      ros2 run object_query object_query_server"
echo "Navigation pane:   top-right     ros2 run kachaka_nav modular_nav_node"
echo "Agent pane:        bottom-left   ros2 run decision_maker agent_decision_maker_node"
echo "NL command pane:   bottom-right  ros2 run decision_maker nl_command_node"
echo "Planner backend:   ${PLANNER_BACKEND}"
if [ "${PLANNER_BACKEND}" = "vllm" ]; then
  echo "vLLM base URL:    ${VLLM_BASE_URL}"
  echo "vLLM model:       ${VLLM_MODEL}"
  echo "Max new tokens:   ${QWEN_MAX_NEW_TOKENS}"
  echo "Temperature:      ${VLLM_TEMPERATURE}"
fi
echo "Kachaka endpoint:  ${KACHAKA_ENDPOINT}"
echo "Agent replans:     ${AGENT_MAX_REPLANS}"
echo "Mouse scroll:      enabled in ${TMUX_CONF}"
