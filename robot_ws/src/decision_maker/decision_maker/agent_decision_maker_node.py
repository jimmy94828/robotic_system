import json
import math
import random
import threading
import time
from datetime import datetime
from pathlib import Path

import rclpy
from kachaka_interfaces.action import Navigate
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String

from .decision_agent import DecisionAgent, PlanValidationError, QwenClient
from .decision_harness import LightweightPlanningHarness
from .decision_maker_node import DecisionMakingNode
from .task_decomposer import DecomposerClient, TaskDecomposer, TaskDecompositionError


class AgentDecisionMakingNode(DecisionMakingNode):
    """Decision maker variant that uses closed-loop agent decisions."""

    def __init__(self):
        super().__init__(node_name="agent_decision_making_node")
        self.llm_client = QwenClient()
        self.agent = DecisionAgent(self.llm_client)
        self.task_decomposer = TaskDecomposer(DecomposerClient(self.llm_client))
        self.declare_parameter("preload_llm_model", True)
        self.preload_llm_model = (
            self.get_parameter("preload_llm_model").get_parameter_value().bool_value
        )
        self.declare_parameter("mock_execution", False)
        self.mock_execution = (
            self.get_parameter("mock_execution").get_parameter_value().bool_value
        )
        self.declare_parameter("enable_task_decomposition", True)
        self.enable_task_decomposition = (
            self.get_parameter("enable_task_decomposition").get_parameter_value().bool_value
        )
        self.declare_parameter("agent_max_steps", 25)
        self.agent_max_steps = (
            self.get_parameter("agent_max_steps").get_parameter_value().integer_value
        )
        self.declare_parameter("agent_max_replans", 2)
        self.agent_max_replans = (
            self.get_parameter("agent_max_replans").get_parameter_value().integer_value
        )
        self.declare_parameter("recover_grasp_failure_with_realign", True)
        self.recover_grasp_failure_with_realign = (
            self.get_parameter("recover_grasp_failure_with_realign")
            .get_parameter_value()
            .bool_value
        )
        self.declare_parameter("grasp_realign_max_attempts", 1)
        self.grasp_realign_max_attempts = (
            self.get_parameter("grasp_realign_max_attempts")
            .get_parameter_value()
            .integer_value
        )
        self.declare_parameter("mock_object_query", True)
        self.mock_object_query = (
            self.get_parameter("mock_object_query").get_parameter_value().bool_value
        )
        self.declare_parameter("mock_failure_rate", 0.0)
        self.mock_failure_rate = (
            self.get_parameter("mock_failure_rate").get_parameter_value().double_value
        )
        self.declare_parameter("mock_failure_seed", 0)
        self.mock_failure_seed = (
            self.get_parameter("mock_failure_seed").get_parameter_value().integer_value
        )
        self.declare_parameter("mock_fail_once_capabilities", "")
        fail_once_text = (
            self.get_parameter("mock_fail_once_capabilities").get_parameter_value().string_value
        )
        self.mock_fail_once_capabilities = {
            item.strip() for item in fail_once_text.split(",") if item.strip()
        }
        self._mock_failed_once = set()
        self._mock_rng = random.Random(self.mock_failure_seed)
        self.harness = LightweightPlanningHarness(
            max_replans_per_task=self.agent_max_replans
        )
        self.declare_parameter("enable_task_report", True)
        self.enable_task_report = (
            self.get_parameter("enable_task_report").get_parameter_value().bool_value
        )
        self.declare_parameter("task_report_dir", "/robot_ws/task_reports")
        self.task_report_dir = (
            self.get_parameter("task_report_dir").get_parameter_value().string_value
        )
        self.task_reply_pub = self.create_publisher(String, "/task_reply", 10)
        self.declare_parameter("robot_pose_global_frame", "map")
        self.declare_parameter("robot_pose_base_frame", "base_link")
        self.declare_parameter("robot_pose_timeout_sec", 1.0)
        self.robot_pose_global_frame = (
            self.get_parameter("robot_pose_global_frame").get_parameter_value().string_value
        )
        self.robot_pose_base_frame = (
            self.get_parameter("robot_pose_base_frame").get_parameter_value().string_value
        )
        self.robot_pose_timeout_sec = (
            self.get_parameter("robot_pose_timeout_sec").get_parameter_value().double_value
        )
        self.tf_buffer = None
        self.tf_listener = None
        if not self.mock_execution:
            try:
                from tf2_ros import Buffer, TransformListener

                self.tf_buffer = Buffer()
                self.tf_listener = TransformListener(self.tf_buffer, self)
            except Exception as exc:
                self.get_logger().warn(f"⚠️ Robot pose TF listener unavailable: {exc}")
        self._preload_llm_model_if_requested()
        mode = "mock execution" if self.mock_execution else "live execution"
        object_query_mode = "mock" if self.mock_execution and self.mock_object_query else "live"
        self.get_logger().info(
            f"🧠 AgentDecisionMakingNode ready with closed-loop planner "
            f"({mode}, object_query={object_query_mode}, harness replans={self.agent_max_replans})."
        )

    def publish_task_reply(
        self,
        task_instruction: str,
        success: bool,
        message: str,
        **extra,
    ) -> None:
        payload = {
            "task": task_instruction,
            "success": bool(success),
            "status": "success" if success else "failed",
            "message": message,
        }
        payload.update(extra)
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.task_reply_pub.publish(msg)
        self.get_logger().info("📤 Task reply: " + msg.data)

    def _new_task_report(self, task_instruction: str) -> dict:
        return {
            "task": task_instruction,
            "started_at": datetime.now().isoformat(timespec="seconds"),
            "_started_monotonic": time.monotonic(),
            "success": None,
            "status": "running",
            "message": "",
            "decomposition": None,
            "reasoning_events": [],
            "decisions": [],
            "subtasks": [],
        }

    def _record_reasoning_event(
        self,
        report: dict | None,
        stage: str,
        duration_sec: float,
        success: bool,
        **extra,
    ) -> None:
        if report is None:
            return
        event = {
            "stage": stage,
            "duration_sec": round(float(duration_sec), 3),
            "success": bool(success),
        }
        event.update(extra)
        report.setdefault("reasoning_events", []).append(event)

    def _write_task_report(self, report: dict | None, success: bool, message: str) -> None:
        if not self.enable_task_report or report is None:
            return

        total_duration = time.monotonic() - float(
            report.get("_started_monotonic", time.monotonic())
        )
        output = dict(report)
        output.pop("_started_monotonic", None)
        output.update({
            "finished_at": datetime.now().isoformat(timespec="seconds"),
            "total_duration_sec": round(total_duration, 3),
            "success": bool(success),
            "status": "success" if success else "failed",
            "message": message,
        })

        try:
            report_dir = Path(self.task_report_dir)
            report_dir.mkdir(parents=True, exist_ok=True)
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            safe_task = "".join(
                char if char.isalnum() else "_"
                for char in report.get("task", "task").lower()
            ).strip("_")
            safe_task = "_".join(part for part in safe_task.split("_") if part)[:60] or "task"
            path = report_dir / f"{timestamp}_{safe_task}.json"
            path.write_text(
                json.dumps(output, ensure_ascii=False, indent=2),
                encoding="utf-8",
            )
            self.get_logger().info(f"📝 Task report written: {path}")
        except Exception as exc:
            self.get_logger().warn(f"⚠️ Failed to write task report: {exc}")

    def _preload_llm_model_if_requested(self) -> None:
        if not self.preload_llm_model:
            self.get_logger().info("🧠 LLM model preload disabled by parameter.")
            return

        if self.llm_client.backend != "local_transformers":
            self.get_logger().info(
                f"🧠 LLM backend '{self.llm_client.backend}' does not require startup model preload."
            )
            return

        self.get_logger().info(
            f"🧠 Preloading local Qwen model at node startup: {self.llm_client.model_path}"
        )
        start_time = time.monotonic()
        try:
            self.llm_client.preload_model()
        except Exception as exc:
            self.get_logger().error(f"❌ Failed to preload local Qwen model: {exc}")
            return

        elapsed = time.monotonic() - start_time
        self.get_logger().info(f"✅ Local Qwen model preloaded in {elapsed:.1f}s.")

    def on_text_event(self, msg: String):
        text = msg.data.strip()
        if not text:
            self.get_logger().warn("⚠️ Received empty manual command. Ignoring.")
            return

        self.get_logger().info(f"📥 Received agent command: '{text}'")
        thread = threading.Thread(
            target=self.run_decomposed_task_loop,
            args=(text,),
            daemon=True,
        )
        thread.start()

    def run_decomposed_task_loop(self, task_instruction: str) -> bool:
        report = self._new_task_report(task_instruction)
        if not self.enable_task_decomposition:
            success = self.run_iterative_task_loop(task_instruction, report=report)
            message = "task completed" if success else "task failed"
            self.publish_task_reply(
                task_instruction,
                success,
                message,
                decomposition_enabled=False,
            )
            self._write_task_report(report, success, message)
            return success

        decomposition_context = self.harness.build_decomposition_context(
            task_instruction=task_instruction,
            supported_capabilities=self._build_agent_context()["supported_capabilities"],
            mock_execution=self.mock_execution,
            extra_context={
                "robot_pose_frames": {
                    "global_frame": self.robot_pose_global_frame,
                    "base_frame": self.robot_pose_base_frame,
                }
            },
        )
        self.get_logger().info("🧩 Harness decomposition context prepared.")

        decomposition = None
        last_decomposition_error = None
        for attempt in range(1, self.agent_max_replans + 2):
            if last_decomposition_error:
                decomposition_context["last_decomposition_failure"] = last_decomposition_error
                self.get_logger().warn(
                    f"🔁 Retrying task decomposition "
                    f"({attempt}/{self.agent_max_replans + 1}): "
                    f"{last_decomposition_error.get('message', 'unknown error')}"
                )
            try:
                reasoning_start = time.monotonic()
                candidate = self.task_decomposer.decompose(
                    task_instruction,
                    context=decomposition_context,
                )
                self._record_reasoning_event(
                    report,
                    "task_decomposition",
                    time.monotonic() - reasoning_start,
                    True,
                    attempt=attempt,
                )
            except (TaskDecompositionError, Exception) as exc:
                self._record_reasoning_event(
                    report,
                    "task_decomposition",
                    time.monotonic() - reasoning_start,
                    False,
                    attempt=attempt,
                    error=str(exc),
                )
                last_decomposition_error = {
                    "stage": "task_decomposition",
                    "success": False,
                    "message": str(exc),
                }
                continue

            verified, verification_message = self.harness.verify_decomposition(
                task_instruction,
                candidate,
            )
            self.get_logger().info(
                "🧩 Task decomposition: " + json.dumps(candidate, ensure_ascii=False)
            )
            report["decomposition"] = candidate
            if verified:
                decomposition = candidate
                self.get_logger().info(
                    f"🧪 Harness decomposition verification: {verification_message}"
                )
                break

            last_decomposition_error = {
                "stage": "task_decomposition_verification",
                "success": False,
                "message": verification_message,
                "rejected_decomposition": candidate,
            }

        if decomposition is None:
            message = (last_decomposition_error or {}).get("message", "unknown error")
            self.get_logger().error(
                f"🛑 Task decomposition failed after replanning attempts: {message}"
            )
            self.publish_task_reply(
                task_instruction,
                False,
                message,
                stage="task_decomposition",
                error=last_decomposition_error,
            )
            self._write_task_report(report, False, message)
            return False

        subtasks = decomposition["subtasks"]
        for subtask in subtasks:
            subtask_id = subtask["subtask_id"]
            subtask_text = self._subtask_text_for_agent(subtask)
            original_subtask_text = subtask.get("text", subtask_text)
            if subtask_text != original_subtask_text:
                self.get_logger().info(
                    f"🧩 Subtask {subtask_id} grounded with metadata: "
                    f"'{original_subtask_text}' -> '{subtask_text}'"
                )
            self.get_logger().info(
                f"🧩 Running subtask {subtask_id}/{len(subtasks)}: {subtask_text}"
            )
            subtask_report = {
                "subtask_id": subtask_id,
                "text": subtask_text,
                "original_text": original_subtask_text,
                "started_at": datetime.now().isoformat(timespec="seconds"),
                "_started_monotonic": time.monotonic(),
                "success": None,
                "duration_sec": None,
            }
            report.setdefault("subtasks", []).append(subtask_report)
            success = self.run_iterative_task_loop(
                subtask_text,
                report=report,
                subtask_report=subtask_report,
            )
            subtask_report["success"] = bool(success)
            subtask_report["finished_at"] = datetime.now().isoformat(timespec="seconds")
            subtask_report["duration_sec"] = round(
                time.monotonic() - float(subtask_report.pop("_started_monotonic")),
                3,
            )
            if not success:
                message = f"Subtask {subtask_id} failed: {subtask_text}"
                self.get_logger().error(
                    f"🛑 {message}; stopping original task '{task_instruction}'."
                )
                self.publish_task_reply(
                    task_instruction,
                    False,
                    message,
                    failed_subtask={
                        "subtask_id": subtask_id,
                        "text": subtask_text,
                        "original_text": original_subtask_text,
                    },
                    decomposition=decomposition,
                )
                self._write_task_report(report, False, message)
                return False
            self.get_logger().info(f"✅ Subtask {subtask_id} completed: {subtask_text}")

        self.get_logger().info(f"✅ All subtasks completed for: {task_instruction}")
        self.publish_task_reply(
            task_instruction,
            True,
            "all subtasks completed",
            decomposition=decomposition,
        )
        self._write_task_report(report, True, "all subtasks completed")
        return True

    def _subtask_text_for_agent(self, subtask: dict) -> str:
        """Preserve decomposer metadata when sending atomic tasks to the agent."""
        text = str(subtask.get("text") or "").strip()
        subtask_type = str(subtask.get("type") or "").strip().lower()
        obj = subtask.get("object")
        source = subtask.get("source")
        destination = subtask.get("destination")

        if isinstance(obj, str):
            obj = obj.strip().lower() or None
        else:
            obj = None
        if isinstance(source, str):
            source = source.strip().lower() or None
        else:
            source = None
        if isinstance(destination, str):
            destination = destination.strip().lower() or None
        else:
            destination = None

        text_lower = text.lower()
        is_handover = subtask_type == "handover" or text_lower.startswith("handover ")

        if obj and source and destination:
            if is_handover:
                return f"handover {obj} on {source} to {destination}"
            if subtask_type == "move":
                return f"move {obj} from {source} to {destination}"
            if subtask_type in {"bring", "conditional"}:
                return f"bring {obj} on {source} to {destination}"
        if obj and destination:
            if is_handover:
                return f"handover {obj} to {destination}"
            if subtask_type == "bring":
                return f"bring {obj} to {destination}"
        return text

    def run_iterative_task_loop(
        self,
        task_instruction: str,
        report: dict | None = None,
        subtask_report: dict | None = None,
    ) -> bool:
        current_state = self.harness.build_initial_state(
            task_instruction=task_instruction,
            supported_capabilities=self._build_agent_context()["supported_capabilities"],
            mock_execution=self.mock_execution,
            extra_context={
                "robot_pose_frames": {
                    "global_frame": self.robot_pose_global_frame,
                    "base_frame": self.robot_pose_base_frame,
                }
            },
        )
        history = []
        last_result = None

        self.get_logger().info("🧩 Harness feedforward context attached to agent state.")

        for step_index in range(1, self.agent_max_steps + 1):
            current_state["step_index"] = step_index
            forced_call = current_state.pop("force_next_capability_call", None)
            if forced_call:
                capability_call = forced_call
                self.get_logger().warn(
                    "🔁 Forced recovery retry capability call: "
                    + json.dumps(capability_call, ensure_ascii=False)
                )
            else:
                decision_state = self.harness.prepare_decision_state(
                    current_state,
                    history,
                    last_result,
                )
                try:
                    reasoning_start = time.monotonic()
                    capability_call = self.agent.decide_next_capability(
                        task_instruction,
                        decision_state,
                        history,
                        last_result,
                    )
                    decision_reasoning_sec = time.monotonic() - reasoning_start
                    self._record_reasoning_event(
                        report,
                        "capability_decision",
                        decision_reasoning_sec,
                        True,
                        subtask=task_instruction,
                        step=step_index,
                    )
                except (PlanValidationError, Exception) as exc:
                    self._record_reasoning_event(
                        report,
                        "capability_decision",
                        time.monotonic() - reasoning_start,
                        False,
                        subtask=task_instruction,
                        step=step_index,
                        error=str(exc),
                    )
                    last_result = {
                        "last_action": "agent_decision",
                        "success": False,
                        "message": str(exc),
                    }
                    self.harness.record_observation(
                        current_state,
                        {"capability": "agent_decision", "step": step_index},
                        last_result,
                    )
                    if self.harness.can_replan_after_failure(current_state):
                        used = self.harness.mark_replan_used(current_state)
                        self.get_logger().warn(
                            f"🔁 Agent decision failed; feeding error back for replan "
                            f"({used}/{self.agent_max_replans}): {exc}"
                        )
                        continue
                    self.get_logger().error(
                        f"❌ Agent decision failed after replanning at step {step_index}: {exc}"
                    )
                    return False
            if forced_call:
                decision_reasoning_sec = 0.0
            verification_context = self.harness.build_verification_context(
                capability_call,
                current_state,
                history,
            )
            verified, verification_message = self.harness.verify_capability_call(
                capability_call,
                current_state,
                history,
            )
            self.get_logger().info(
                "🧠 Agent capability call: "
                + json.dumps(capability_call, ensure_ascii=False)
            )
            decision_entry = {
                "subtask": task_instruction,
                "step": step_index,
                "capability_call": capability_call,
                "reasoning_duration_sec": round(float(decision_reasoning_sec), 3),
                "forced": bool(forced_call),
                "verified": bool(verified),
                "verification_message": verification_message,
            }
            if report is not None:
                report.setdefault("decisions", []).append(decision_entry)
                if subtask_report is not None:
                    subtask_report.setdefault("decision_indexes", []).append(
                        len(report.get("decisions", [])) - 1
                    )
            if not verified:
                last_result = {
                    "last_action": "harness_verification",
                    "success": False,
                    "message": verification_message,
                    "rejected_call": capability_call,
                    "verification_context": verification_context,
                }
                decision_entry["result"] = last_result
                self.harness.record_observation(current_state, capability_call, last_result)
                if self.harness.can_replan_after_failure(current_state):
                    used = self.harness.mark_replan_used(current_state)
                    self.get_logger().warn(
                        f"🧪 Harness rejected call and requested replan "
                        f"({used}/{self.agent_max_replans}): {verification_message}"
                    )
                    continue
                self.get_logger().error(f"🛑 Harness rejected call: {verification_message}")
                return False

            self.get_logger().info(
                f"🧪 Harness verification ({verification_context['stage']}): {verification_message}"
            )

            if capability_call["capability"] == "finish":
                decision_entry["result"] = {
                    "success": True,
                    "message": capability_call.get("reason", "finish"),
                }
                self.get_logger().info(
                    f"✅ Agent finished task '{task_instruction}': "
                    f"{capability_call.get('reason', '')}"
                )
                return True

            result = self.execute_capability_call(capability_call, current_state)
            decision_entry["result"] = result
            self.harness.record_observation(current_state, capability_call, result)
            last_result = result

            self.get_logger().info(
                "👁️ Capability observation: " + json.dumps(result, ensure_ascii=False)
            )

            if not result.get("success", False):
                recovery_result = self._try_recover_grasp_failure_with_realign(
                    current_state,
                    history,
                    capability_call,
                    result,
                )
                if recovery_result is not None:
                    history.append({"step": step_index, "call": capability_call, "result": result})
                    recovery_call = recovery_result["call"]
                    recovery_observation = recovery_result["result"]
                    if report is not None:
                        recovery_decision = {
                            "subtask": task_instruction,
                            "step": step_index,
                            "capability_call": recovery_call,
                            "reasoning_duration_sec": 0.0,
                            "forced": True,
                            "verified": True,
                            "verification_message": "automatic grasp-failure realignment recovery",
                            "result": recovery_observation,
                        }
                        report.setdefault("decisions", []).append(recovery_decision)
                        if subtask_report is not None:
                            subtask_report.setdefault("decision_indexes", []).append(
                                len(report.get("decisions", [])) - 1
                            )
                    self.harness.record_observation(
                        current_state,
                        recovery_call,
                        recovery_observation,
                    )
                    history.append({
                        "step": step_index,
                        "call": recovery_call,
                        "result": recovery_observation,
                    })
                    last_result = recovery_observation
                    self.get_logger().info(
                        "👁️ Capability observation: "
                        + json.dumps(recovery_observation, ensure_ascii=False)
                    )
                    if recovery_observation.get("success"):
                        current_state["force_next_capability_call"] = dict(capability_call)
                        self.get_logger().warn(
                            "🔁 Realignment completed; forcing retry of the failed grasp before any destination navigation."
                        )
                    continue

                if (
                    capability_call.get("capability") == "grasp_place"
                    and capability_call.get("action") == "grasp"
                ):
                    history.append({"step": step_index, "call": capability_call, "result": result})
                    self.get_logger().error(
                        f"🛑 Grasp failed after recovery/retry; object "
                        f"'{capability_call.get('target')}' is not held. "
                        f"Stopping subtask instead of navigating to a destination. "
                        f"Reason: {result.get('message', 'unknown error')}"
                    )
                    return False

                if self.harness.can_replan_after_failure(current_state):
                    used = self.harness.mark_replan_used(current_state)
                    self.get_logger().warn(
                        f"🔁 Capability failed; feeding observation back for replan "
                        f"({used}/{self.agent_max_replans}): "
                        f"{result.get('message', 'unknown error')}"
                    )
                    continue

                history.append({"step": step_index, "call": capability_call, "result": result})
                self.get_logger().error(
                    f"🛑 Stopping task '{task_instruction}' after failed capability: "
                    f"{result.get('message', 'unknown error')}"
                )
                return False

            history.append({"step": step_index, "call": capability_call, "result": result})

        self.get_logger().error(
            f"⏰ Agent task '{task_instruction}' exceeded max_steps={self.agent_max_steps}."
        )
        return False

    def _try_recover_grasp_failure_with_realign(
        self,
        current_state: dict,
        history: list,
        failed_call: dict,
        failed_result: dict,
    ) -> dict | None:
        if not self.recover_grasp_failure_with_realign:
            return None
        if failed_call.get("capability") != "grasp_place":
            return None
        if failed_call.get("action") != "grasp":
            return None

        message = str(failed_result.get("message", "")).lower()
        camera_like_failure = any(
            token in message
            for token in (
                "image",
                "camera",
                "rgb",
                "depth",
                "not found",
                "not detected",
                "no synchronized",
                "view",
                "visible",
                "grasp failed",
            )
        )
        if message and not camera_like_failure:
            return None

        target_object = failed_call.get("target", "unknown")
        recovery_counts = current_state.setdefault("grasp_realign_recovery_counts", {})
        attempts_used = int(recovery_counts.get(target_object, 0))
        if attempts_used >= self.grasp_realign_max_attempts:
            self.get_logger().warn(
                f"🔁 Grasp realignment already attempted for {target_object} "
                f"({attempts_used}/{self.grasp_realign_max_attempts}); returning to normal replanning."
            )
            return None

        nav_entry = self._last_successful_navigation_entry(history)
        if nav_entry is None:
            self.get_logger().warn(
                "🔁 Grasp failed, but no successful navigation target is available for realignment."
            )
            return None

        nav_call = nav_entry["call"]
        nav_result = nav_entry["result"]
        target = nav_result.get("target") or nav_call.get("target")
        pose = nav_result.get("pose") or nav_call.get("pose")
        if pose is None and target:
            pose = current_state.get("known_poses", {}).get(target)

        recovery_call = {
            "capability": "navigation",
            "target": target,
            "reason": "Recover from grasp failure by realigning the robot camera toward the last grasp source.",
        }
        if pose:
            recovery_call["pose"] = pose

        recovery_counts[target_object] = attempts_used + 1
        self.get_logger().warn(
            "🔁 Grasp failed; realigning at last navigation target before retrying grasp. "
            f"target={target}, object={target_object}, "
            f"attempt={recovery_counts[target_object]}/{self.grasp_realign_max_attempts}, "
            f"message={failed_result.get('message', 'unknown error')}"
        )
        recovery_observation = self.execute_navigation(
            target=target,
            pose=pose,
            current_state=current_state,
        )
        recovery_observation["recovery_for"] = "grasp_failure"
        return {"call": recovery_call, "result": recovery_observation}

    def _last_successful_navigation_entry(self, history: list) -> dict | None:
        for entry in reversed(history):
            call = entry.get("call", {})
            result = entry.get("result", {})
            if call.get("capability") == "navigation" and result.get("success"):
                return entry
        return None

    def execute_capability_call(self, capability_call: dict, current_state: dict) -> dict:
        capability = capability_call["capability"]
        if capability == "object_query":
            return self.execute_object_query(capability_call["target"], current_state)
        if capability == "navigation":
            return self.execute_navigation(
                target=capability_call.get("target"),
                pose=capability_call.get("pose"),
                current_state=current_state,
            )
        if capability == "grasp_place":
            return self.execute_grasp_place(
                action=capability_call["action"],
                target=capability_call["target"],
                destination=capability_call.get("destination"),
                current_state=current_state,
            )
        if capability == "robot_pose":
            return self.execute_robot_pose(current_state)
        return {
            "last_action": capability,
            "success": False,
            "message": f"Unsupported capability: {capability}",
        }

    def execute_robot_pose(self, current_state: dict) -> dict:
        if self.mock_execution:
            pose = {"x": 0.0, "y": 0.0, "theta": 0.0, "frame": "map"}
            current_state["robot_pose"] = pose
            self.get_logger().info(f"🧪📍 [MOCK] robot_pose() -> {pose}")
            return {
                "last_action": "robot_pose",
                "success": True,
                "result": {"pose": pose},
                "message": "mock robot pose completed",
            }

        if self.tf_buffer is None:
            return {
                "last_action": "robot_pose",
                "success": False,
                "message": "TF listener is not available for robot pose lookup",
            }

        try:
            from rclpy.duration import Duration
            from rclpy.time import Time

            transform = self.tf_buffer.lookup_transform(
                self.robot_pose_global_frame,
                self.robot_pose_base_frame,
                Time(),
                timeout=Duration(seconds=float(self.robot_pose_timeout_sec)),
            )
            translation = transform.transform.translation
            rotation = transform.transform.rotation
            theta = self._yaw_from_quaternion(rotation)
            pose = {
                "x": float(translation.x),
                "y": float(translation.y),
                "z": float(translation.z),
                "theta": float(theta),
                "frame": self.robot_pose_global_frame,
                "child_frame": self.robot_pose_base_frame,
            }
            current_state["robot_pose"] = pose
            self.get_logger().info(
                f"📍 Robot pose: ({pose['x']:.2f}, {pose['y']:.2f}, {pose['theta']:.2f}) "
                f"in {self.robot_pose_global_frame}"
            )
            return {
                "last_action": "robot_pose",
                "success": True,
                "result": {"pose": pose},
                "message": "robot pose lookup completed",
            }
        except Exception as exc:
            return {
                "last_action": "robot_pose",
                "success": False,
                "message": f"robot pose lookup failed: {exc}",
            }

    def execute_object_query(self, target: str, current_state: dict) -> dict:
        if self.mock_execution and self.mock_object_query:
            failure = self._maybe_mock_failure("object_query", target)
            if failure:
                self.get_logger().warn(
                    "🧪💥 [MOCK] object_query failure -> "
                    + json.dumps(failure, ensure_ascii=False)
                )
                return failure
            pose = self._mock_pose_for_target(target)
            current_state.setdefault("known_poses", {})[target] = pose
            self.get_logger().info(f"🧪🔎 [MOCK] object_query({target}) -> {pose}")
            return {
                "last_action": "object_query",
                "target": target,
                "success": True,
                "result": {"pose": pose},
                "message": "mock object query completed",
            }

        pose_tuple = self._query_object_position(target)
        if not pose_tuple:
            return {
                "last_action": "object_query",
                "target": target,
                "success": False,
                "message": f"Object or place '{target}' was not found.",
            }

        pose = {
            "x": float(pose_tuple[0]),
            "y": float(pose_tuple[1]),
            "theta": float(pose_tuple[2]) if len(pose_tuple) > 2 else 0.0,
        }
        current_state.setdefault("known_poses", {})[target] = pose
        return {
            "last_action": "object_query",
            "target": target,
            "success": True,
            "result": {"pose": pose},
        }

    def execute_navigation(
        self,
        target: str | None = None,
        pose: dict | None = None,
        current_state: dict | None = None,
    ) -> dict:
        if pose is None and target and current_state is not None:
            pose = current_state.get("known_poses", {}).get(target)
            if pose:
                self.get_logger().info(
                    f"📌 Using cached pose for navigation target '{target}'."
                )

        if self.mock_execution:
            failure = self._maybe_mock_failure("navigation", target)
            if failure:
                if pose:
                    failure["pose"] = pose
                self.get_logger().warn(
                    "🧪💥 [MOCK] navigation failure -> "
                    + json.dumps(failure, ensure_ascii=False)
                )
                return failure
            result = {
                "last_action": "navigation",
                "success": True,
                "message": "mock navigation completed",
            }
            if target:
                result["target"] = target
            if pose:
                result["pose"] = pose
            self.get_logger().info(
                "🧪🧭 [MOCK] navigation -> " + json.dumps(result, ensure_ascii=False)
            )
            return result

        if pose:
            success = self._execute_nav_pose(pose)
            result = {
                "last_action": "navigation",
                "pose": pose,
                "success": bool(success),
                "message": "navigation completed" if success else "navigation failed",
            }
            if target:
                result["target"] = target
            return result

        if target:
            success = self._execute_nav(f"goto:{target}")
            return {
                "last_action": "navigation",
                "target": target,
                "success": bool(success),
                "message": "navigation completed" if success else "navigation failed",
            }

        return {
            "last_action": "navigation",
            "success": False,
            "message": "navigation requires target or pose",
        }

    def execute_grasp_place(
        self,
        action: str,
        target: str,
        destination: str | None = None,
        current_state: dict | None = None,
    ) -> dict:
        if self.mock_execution:
            failure = self._maybe_mock_failure("grasp_place", target, action)
            if failure:
                if destination:
                    failure["destination"] = destination
                self.get_logger().warn(
                    "🧪💥 [MOCK] grasp_place failure -> "
                    + json.dumps(failure, ensure_ascii=False)
                )
                return failure
            if action == "grasp" and current_state is not None:
                current_state["held_object"] = target
            elif action in {"place", "handover"} and current_state is not None:
                current_state["held_object"] = None
            result = {
                "last_action": "grasp_place",
                "action": action,
                "target": target,
                "success": action in {"grasp", "place", "handover"},
                "message": (
                    f"mock {action} completed"
                    if action in {"grasp", "place", "handover"}
                    else f"unsupported mock action: {action}"
                ),
            }
            if destination:
                result["destination"] = destination
            self.get_logger().info(
                "🧪🦾 [MOCK] grasp_place -> " + json.dumps(result, ensure_ascii=False)
            )
            return result

        if action == "grasp":
            success = self._execute_grasp(f"grasp:{target}")
            if success and current_state is not None:
                current_state["held_object"] = target
            return {
                "last_action": "grasp_place",
                "action": "grasp",
                "target": target,
                "success": bool(success),
                "message": "grasp completed" if success else "grasp failed",
            }

        if action == "place":
            cmd = f"place:{target}:{destination}"
            success = self._execute_place(cmd)
            if success and current_state is not None:
                current_state["held_object"] = None
            return {
                "last_action": "grasp_place",
                "action": "place",
                "target": target,
                "destination": destination,
                "success": bool(success),
                "message": "place completed" if success else "place failed",
            }

        if action == "handover":
            success = self._execute_handover(f"handover:{target}")
            if success and current_state is not None:
                current_state["held_object"] = None
            result = {
                "last_action": "grasp_place",
                "action": "handover",
                "target": target,
                "success": bool(success),
                "message": "handover completed" if success else "handover failed",
            }
            if destination:
                result["destination"] = destination
            return result

        return {
            "last_action": "grasp_place",
            "action": action,
            "target": target,
            "success": False,
            "message": f"Unsupported grasp_place action: {action}",
        }


    def _maybe_mock_failure(
        self,
        capability: str,
        target: str | None = None,
        action: str | None = None,
    ) -> dict | None:
        if not self.mock_execution:
            return None

        key_parts = [capability]
        if action:
            key_parts.append(action)
        if target:
            key_parts.append(target)
        key = ":".join(key_parts)

        fail_once_key = capability if capability in self.mock_fail_once_capabilities else key
        should_fail_once = (
            fail_once_key in self.mock_fail_once_capabilities
        ) and fail_once_key not in self._mock_failed_once
        should_fail_randomly = (
            self.mock_failure_rate > 0.0
            and self._mock_rng.random() < min(max(self.mock_failure_rate, 0.0), 1.0)
        )

        if not should_fail_once and not should_fail_randomly:
            return None

        if should_fail_once:
            self._mock_failed_once.add(fail_once_key)

        result = {
            "last_action": capability,
            "success": False,
            "message": f"mock injected {capability} failure",
            "mock_failure": True,
        }
        if target:
            result["target"] = target
        if action:
            result["action"] = action
        return result

    def _yaw_from_quaternion(self, q) -> float:
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def _mock_pose_for_target(self, target: str) -> dict:
        mock_poses = {
            "home": {"x": 0.0, "y": 0.0, "theta": 0.0},
            "me": {"x": 0.0, "y": 0.0, "theta": 0.0},
            "cabinet": {"x": 1.0, "y": 0.5, "theta": 0.0},
            "table": {"x": 2.0, "y": 0.0, "theta": 0.0},
            "sofa": {"x": 3.0, "y": 1.0, "theta": 0.0},
            "chair": {"x": 2.5, "y": -1.0, "theta": 0.0},
            "kitchen": {"x": 4.0, "y": 0.0, "theta": 0.0},
            "fridge": {"x": 4.5, "y": 0.5, "theta": 0.0},
        }
        return dict(mock_poses.get(target, {"x": 1.0, "y": 1.0, "theta": 0.0}))

    def _execute_nav_pose(self, pose: dict, timeout_sec: float = 150.0) -> bool:
        try:
            if not self.nav_client.wait_for_server(timeout_sec=10.0):
                self.get_logger().error("❌ Nav server not available.")
                return False

            goal = Navigate.Goal()
            goal.target_x = float(pose["x"])
            goal.target_y = float(pose["y"])
            # Object-query poses do not provide a reliable yaw. NaN asks the
            # navigation node to face the target from the current robot pose.
            goal.target_theta = float(pose.get("target_theta", pose.get("yaw", math.nan)))

            self.get_logger().info(
                f"🧭 Sending Nav Goal from pose: "
                f"({goal.target_x:.2f}, {goal.target_y:.2f}, {goal.target_theta:.2f})"
            )
            fut = self.nav_client.send_goal_async(
                goal,
                feedback_callback=self._on_nav_feedback,
            )
            start = time.time()
            while not fut.done():
                if time.time() - start > 10.0:
                    self.get_logger().error("⏰ NAV goal send timeout")
                    return False
                time.sleep(0.05)

            gh = fut.result()
            if not gh.accepted:
                self.get_logger().warn("⚠️ NAV goal rejected.")
                return False

            res_future = gh.get_result_async()
            start = time.time()
            while not res_future.done():
                if time.time() - start > timeout_sec:
                    self.get_logger().warn("⏰ NAV timeout")
                    self._send_cancel()
                    return False
                time.sleep(0.1)

            nav_result = res_future.result().result
            if not nav_result.success:
                self.get_logger().warn(f"❌ NAV failed: {nav_result.message}")
                return False

            self.get_logger().info("✅ NAV success.")
            return True
        except Exception as exc:
            self.get_logger().error(f"❌ NAV pose error: {exc}")
            return False

    def _build_agent_context(self) -> dict:
        return {
            "supported_capabilities": [
                "object_query",
                "navigation",
                "grasp_place",
                "robot_pose",
                "finish",
            ],
            "execution_note": (
                "Choose only the next capability call. The system executes it and "
                "returns an observation before the next decision."
            ),
        }


def main():
    rclpy.init()
    node = AgentDecisionMakingNode()
    try:
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
