import base64
import json
import math
import random
import threading
import time
import urllib.request
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
        self.declare_parameter("recover_grasp_failure_with_realign", False)
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
        self.declare_parameter("enable_pre_action_view_check", True)
        self.enable_pre_action_view_check = (
            self.get_parameter("enable_pre_action_view_check").get_parameter_value().bool_value
        )
        # Backward-compatible alias for older launch files/params.
        self.declare_parameter("enable_pre_grasp_view_check", self.enable_pre_action_view_check)
        self.enable_pre_action_view_check = (
            self.enable_pre_action_view_check
            and self.get_parameter("enable_pre_grasp_view_check").get_parameter_value().bool_value
        )
        self.declare_parameter("view_check_max_adjustments", 3)
        self.view_check_max_adjustments = max(
            0,
            self.get_parameter("view_check_max_adjustments")
            .get_parameter_value()
            .integer_value,
        )
        self.declare_parameter("view_critic_backend", "auto")
        self.view_critic_backend = (
            self.get_parameter("view_critic_backend").get_parameter_value().string_value
        ).strip().lower()
        if self.view_critic_backend == "auto":
            self.view_critic_backend = "mock" if self.mock_execution else "vllm"
        self.declare_parameter("view_critic_vllm_base_url", "http://localhost:8001/v1")
        self.view_critic_vllm_base_url = (
            self.get_parameter("view_critic_vllm_base_url")
            .get_parameter_value()
            .string_value
        ).rstrip("/")
        self.declare_parameter("view_critic_vllm_model", "qwen-vl")
        self.view_critic_vllm_model = (
            self.get_parameter("view_critic_vllm_model")
            .get_parameter_value()
            .string_value
        )
        self.declare_parameter("view_critic_max_tokens", 256)
        self.view_critic_max_tokens = (
            self.get_parameter("view_critic_max_tokens").get_parameter_value().integer_value
        )
        self.declare_parameter("view_critic_temperature", 0.0)
        self.view_critic_temperature = (
            self.get_parameter("view_critic_temperature").get_parameter_value().double_value
        )
        self.declare_parameter("view_critic_timeout_sec", 20.0)
        self.view_critic_timeout_sec = (
            self.get_parameter("view_critic_timeout_sec").get_parameter_value().double_value
        )
        self.declare_parameter("view_critic_image_topic", "/camera/color/image_raw")
        self.view_critic_image_topic = (
            self.get_parameter("view_critic_image_topic").get_parameter_value().string_value
        )
        self.declare_parameter("view_adjust_min_yaw_deg", 10.0)
        self.view_adjust_min_yaw_deg = max(
            0.0,
            self.get_parameter("view_adjust_min_yaw_deg").get_parameter_value().double_value,
        )
        self.declare_parameter("view_adjust_scan_step_deg", 15.0)
        self.view_adjust_scan_step_deg = max(
            self.view_adjust_min_yaw_deg,
            abs(
                self.get_parameter("view_adjust_scan_step_deg")
                .get_parameter_value()
                .double_value
            ),
        )
        self.declare_parameter("view_adjust_max_yaw_deg", 60.0)
        self.view_adjust_max_yaw_deg = (
            self.get_parameter("view_adjust_max_yaw_deg").get_parameter_value().double_value
        )
        self.declare_parameter("view_adjust_max_forward_m", 0.25)
        self.view_adjust_max_forward_m = (
            self.get_parameter("view_adjust_max_forward_m").get_parameter_value().double_value
        )
        self.declare_parameter("mock_view_critic_visible_after_attempts", 1)
        self.mock_view_critic_visible_after_attempts = max(
            1,
            self.get_parameter("mock_view_critic_visible_after_attempts")
            .get_parameter_value()
            .integer_value,
        )
        self._mock_view_critic_counts = {}
        self._latest_rgb = None
        self._latest_rgb_stamp = None
        self._latest_rgb_lock = threading.Lock()
        self._rgb_sub = None
        self._setup_view_critic_rgb_subscription()
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

    def _setup_view_critic_rgb_subscription(self) -> None:
        if not self.enable_pre_action_view_check:
            return
        if self.view_critic_backend == "mock":
            self.get_logger().info("👁️ Pre-grasp view critic enabled with mock backend.")
            return
        try:
            from sensor_msgs.msg import Image

            self._rgb_sub = self.create_subscription(
                Image,
                self.view_critic_image_topic,
                self._view_critic_rgb_cb,
                10,
            )
            self.get_logger().info(
                f"👁️ Pre-action view critic subscribed to RGB topic: "
                f"{self.view_critic_image_topic}"
            )
        except Exception as exc:
            self.get_logger().warn(
                f"⚠️ Failed to set up RGB subscription for pre-action view critic: {exc}"
            )

    def _view_critic_rgb_cb(self, msg) -> None:
        try:
            rgb = self._ros_image_to_rgb_array(msg)
        except Exception as exc:
            self.get_logger().warn(f"⚠️ Failed to decode RGB image for view critic: {exc}")
            return
        with self._latest_rgb_lock:
            self._latest_rgb = rgb
            self._latest_rgb_stamp = self.get_clock().now().to_msg()

    def _ros_image_to_rgb_array(self, msg):
        import numpy as np

        encoding = str(msg.encoding).lower()
        height = int(msg.height)
        width = int(msg.width)
        step = int(msg.step)
        if height <= 0 or width <= 0 or step <= 0:
            raise ValueError("invalid ROS image dimensions")

        data = np.frombuffer(msg.data, dtype=np.uint8)
        rows = data.reshape((height, step))

        if encoding in {"rgb8", "bgr8"}:
            channels = 3
            image = rows[:, : width * channels].reshape((height, width, channels))
            if encoding == "bgr8":
                image = image[:, :, ::-1]
            return image.copy()

        if encoding in {"rgba8", "bgra8"}:
            channels = 4
            image = rows[:, : width * channels].reshape((height, width, channels))[:, :, :3]
            if encoding == "bgra8":
                image = image[:, :, ::-1]
            return image.copy()

        if encoding in {"mono8", "8uc1"}:
            image = rows[:, :width].reshape((height, width))
            return np.repeat(image[:, :, None], 3, axis=2).copy()

        raise ValueError(f"unsupported RGB image encoding: {msg.encoding}")

    def _latest_rgb_snapshot(self):
        with self._latest_rgb_lock:
            if self._latest_rgb is None:
                return None
            return self._latest_rgb.copy()

    def publish_task_reply(
        self,
        task_instruction: str,
        success: bool,
        message: str,
        **extra,
    ) -> None:
        payload = {
            "event": "task_result",
            "terminal": True,
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
                        f"🛑 Grasp failed; object "
                        f"'{capability_call.get('target')}' is not held. "
                        f"Stopping subtask immediately. "
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
        if isinstance(failed_result.get("view_check"), dict):
            self.get_logger().warn(
                "🔁 Pre-grasp view check failed; not running grasp realignment recovery "
                "because view adjustment already used its configured budget."
            )
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
        if len(pose_tuple) >= 6:
            # Raw 3D semantic-map center; lets navigation match the instance
            # bbox in the semantic table for occupancy-aware goal selection.
            pose["raw_xyz"] = [
                float(pose_tuple[3]),
                float(pose_tuple[4]),
                float(pose_tuple[5]),
            ]
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
            apply_standoff = bool(target)
            success = self._execute_nav_pose(pose, apply_standoff=apply_standoff)
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
            # _execute_nav resolves the object itself (query + bbox matching
            # + occupancy-aware goal selection).
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
        if action in {"grasp", "place", "handover"} and self.enable_pre_action_view_check:
            view_task = self._view_task_for_grasp_place(action, target, destination)
            view_result = self.ensure_view_sufficient_for_action(view_task, current_state or {})
            if not view_result.get("success"):
                return {
                    "last_action": "grasp_place",
                    "action": action,
                    "target": target,
                    "destination": destination,
                    "success": False,
                    "message": view_result.get(
                        "message",
                        f"view is not sufficient before {action} for {target}",
                    ),
                    "view_check": view_result,
                }

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

    def _view_task_for_grasp_place(
        self,
        action: str,
        target: str,
        destination: str | None,
    ) -> dict:
        action = str(action).strip().lower()
        if action == "grasp":
            return {
                "action": "grasp",
                "target": target,
                "destination": destination,
                "view_target": target,
                "requirement": "target_object_visible",
                "success_condition": (
                    f"Most of the target object '{target}' is visible in the camera view "
                    "and clear enough to localize. It does not need to be perfectly centered "
                    "or optimally close."
                ),
            }
        if action == "place":
            view_target = destination or "placement surface"
            return {
                "action": "place",
                "target": target,
                "destination": destination,
                "view_target": view_target,
                "requirement": "empty_flat_place_surface_visible",
                "success_condition": (
                    f"An empty, reachable, flat placement area on or near '{view_target}' "
                    f"is visible for placing '{target}'."
                ),
            }
        if action == "handover":
            view_target = destination or "human hand"
            return {
                "action": "handover",
                "target": target,
                "destination": destination,
                "view_target": view_target,
                "requirement": "human_hand_visible",
                "success_condition": (
                    "A human hand is visible and positioned for receiving the object."
                ),
            }
        return {
            "action": action,
            "target": target,
            "destination": destination,
            "view_target": target,
            "requirement": "unknown",
            "success_condition": "The required view is sufficient for the action.",
        }

    def ensure_view_sufficient_for_action(self, view_task: dict, current_state: dict) -> dict:
        max_adjustments = int(self.view_check_max_adjustments)
        observations = []
        adjustments_used = 0
        action = view_task.get("action", "action")
        view_target = view_task.get("view_target") or view_task.get("target") or "target"

        for check_index in range(1, max_adjustments + 2):
            rgb = self._latest_rgb_snapshot()
            view = self.call_view_critic(view_task, rgb, check_index, current_state)
            image_path = self._write_view_check_debug_image(
                view_task,
                check_index,
                rgb,
                view,
            )
            if image_path:
                view["debug_image_path"] = image_path

            observations.append(view)
            self.get_logger().info(
                "👁️ View check "
                + json.dumps(
                    {
                        "action": action,
                        "view_target": view_target,
                        "requirement": view_task.get("requirement"),
                        "check": check_index,
                        "sufficient": view.get("sufficient_view"),
                        "adjustment": view.get("adjustment"),
                        "debug_image_path": image_path,
                        "reason": view.get("reason", ""),
                    },
                    ensure_ascii=False,
                )
            )

            view_is_sufficient = bool(view.get("sufficient_view"))

            if view_is_sufficient:
                return {
                    "last_action": "view_critic",
                    "action": action,
                    "target": view_task.get("target"),
                    "destination": view_task.get("destination"),
                    "view_target": view_target,
                    "requirement": view_task.get("requirement"),
                    "success": True,
                    "message": f"view is sufficient before {action}: {view_target}",
                    "adjustments_used": adjustments_used,
                    "observations": observations,
                }

            if adjustments_used >= max_adjustments:
                return {
                    "last_action": "view_critic",
                    "action": action,
                    "target": view_task.get("target"),
                    "destination": view_task.get("destination"),
                    "view_target": view_target,
                    "requirement": view_task.get("requirement"),
                    "success": True,
                    "message": (
                        f"view was still not sufficient after {max_adjustments} "
                        f"adjustments, but proceeding to {action}: {view_target}"
                    ),
                    "view_check_soft_failed": True,
                    "adjustments_used": adjustments_used,
                    "observations": observations,
                }

            adjustment = view.get("adjustment")
            if not adjustment:
                adjustment = self._fallback_view_adjustment(
                    view_task,
                    view,
                    check_index=check_index,
                    adjustments_used=adjustments_used,
                )
                if adjustment:
                    view["adjustment"] = adjustment
                    self.get_logger().warn(
                        "👁️ View critic returned no adjustment; using fallback adjustment: "
                        + json.dumps(adjustment, ensure_ascii=False)
                    )
            if adjustment:
                adjustment = self._shape_view_adjustment_scan(
                    view_task,
                    view,
                    adjustment,
                    adjustments_used=adjustments_used,
                )
                view["adjustment"] = adjustment

            if not adjustment:
                return {
                    "last_action": "view_critic",
                    "action": action,
                    "target": view_task.get("target"),
                    "destination": view_task.get("destination"),
                    "view_target": view_target,
                    "requirement": view_task.get("requirement"),
                    "success": True,
                    "message": (
                        "view critic did not provide an adjustment; "
                        f"proceeding to {action}: {view_target}"
                    ),
                    "view_check_soft_failed": True,
                    "adjustments_used": adjustments_used,
                    "observations": observations,
                }

            nav_result = self.apply_view_adjustment_with_navigation(
                adjustment,
                current_state,
            )
            adjustments_used += 1
            view["navigation_adjustment_result"] = nav_result
            if not nav_result.get("success"):
                return {
                    "last_action": "view_adjustment",
                    "action": action,
                    "target": view_task.get("target"),
                    "destination": view_task.get("destination"),
                    "view_target": view_target,
                    "requirement": view_task.get("requirement"),
                    "success": False,
                    "message": nav_result.get("message", "view adjustment navigation failed"),
                    "adjustments_used": adjustments_used,
                    "observations": observations,
                }

        return {
            "last_action": "view_critic",
            "action": action,
            "target": view_task.get("target"),
            "destination": view_task.get("destination"),
            "view_target": view_target,
            "requirement": view_task.get("requirement"),
            "success": False,
            "message": f"required view for {action} did not become sufficient: {view_target}",
            "adjustments_used": adjustments_used,
            "observations": observations,
        }

    def _fallback_view_adjustment(
        self,
        view_task: dict,
        view: dict,
        check_index: int,
        adjustments_used: int,
    ) -> dict | None:
        action = str(view_task.get("action", "")).strip().lower()
        reason = str(view.get("reason", "")).lower()
        sufficient = bool(view.get("sufficient_view"))

        if sufficient:
            return None

        if action == "grasp":
            if any(token in reason for token in ("far", "small", "distant", "too far")):
                return {"type": "move_forward", "distance_m": 0.15, "fallback": True}
            if any(
                token in reason
                for token in ("close", "too close", "crop", "cropped", "cut off", "partial", "partially")
            ):
                return {"type": "move_backward", "distance_m": 0.15, "fallback": True}

        yaw_scan = self._view_yaw_scan()
        yaw = yaw_scan[adjustments_used % len(yaw_scan)]
        return {"type": "rotate", "yaw_deg": yaw, "fallback": True}

    def _view_yaw_scan(self) -> list[float]:
        """Return small, alternating relative turns that expand the searched view."""
        max_yaw = abs(float(self.view_adjust_max_yaw_deg))
        step = abs(float(self.view_adjust_scan_step_deg))
        if max_yaw <= 0.0:
            return [0.0]
        step = min(max_yaw, max(float(self.view_adjust_min_yaw_deg), step))
        return [
            min(max_yaw, step),
            -min(max_yaw, 2.0 * step),
            min(max_yaw, 3.0 * step),
            -min(max_yaw, 4.0 * step),
        ]

    def _shape_view_adjustment_scan(
        self,
        view_task: dict,
        view: dict,
        adjustment: dict,
        *,
        adjustments_used: int,
    ) -> dict:
        if not isinstance(adjustment, dict):
            return adjustment

        adjustment_type = str(adjustment.get("type", "")).strip().lower()
        if adjustment_type != "rotate":
            return adjustment

        action = str(view_task.get("action", "")).strip().lower()
        reason = str(view.get("reason", "")).strip().lower()
        if action != "grasp":
            return adjustment

        # If the object is partially visible, let the VLM choose a local
        # correction. If it is not clearly partially visible, use an expanding
        # scan pattern even when the VLM repeatedly suggests the same yaw.
        partial_or_local_view = any(
            token in reason
            for token in (
                "partially visible",
                "partial",
                "partly visible",
                "cropped",
                "cut off",
                "edge",
                "too close",
                "too far",
                "small",
                "distant",
            )
        )
        if partial_or_local_view:
            return adjustment

        yaw_scan = self._view_yaw_scan()
        yaw = yaw_scan[adjustments_used % len(yaw_scan)]
        original_yaw = adjustment.get("yaw_deg")
        shaped = dict(adjustment)
        shaped["type"] = "rotate"
        shaped["yaw_deg"] = yaw
        shaped["scan_override"] = True
        shaped["original_yaw_deg"] = original_yaw
        self.get_logger().warn(
            "👁️ Overriding repeated/missing-target rotate with scan pattern: "
            + json.dumps(
                {
                    "original_yaw_deg": original_yaw,
                    "scan_yaw_deg": yaw,
                    "adjustments_used": adjustments_used,
                    "reason": view.get("reason", ""),
                },
                ensure_ascii=False,
            )
        )
        return shaped

    def call_view_critic(
        self,
        view_task: dict,
        rgb_image,
        check_index: int,
        current_state: dict,
    ) -> dict:
        if self.view_critic_backend == "mock":
            return self._mock_view_critic(view_task)
        if self.view_critic_backend == "vllm":
            return self._vllm_view_critic(view_task, rgb_image, check_index, current_state)
        return {
            "sufficient_view": False,
            "confidence": 0.0,
            "reason": f"unsupported view critic backend: {self.view_critic_backend}; use vllm or mock",
            "adjustment": None,
        }

    def _mock_view_critic(self, view_task: dict) -> dict:
        key = ":".join(
            str(part)
            for part in (
                view_task.get("action"),
                view_task.get("view_target"),
                view_task.get("requirement"),
            )
        )
        count = self._mock_view_critic_counts.get(key, 0) + 1
        self._mock_view_critic_counts[key] = count
        visible = count >= self.mock_view_critic_visible_after_attempts
        if visible:
            return {
                "sufficient_view": True,
                "confidence": 1.0,
                "reason": f"mock view critic reports sufficient view for {view_task.get('action')}",
                "adjustment": None,
            }
        yaw = -25.0 if count % 2 == 1 else 25.0
        return {
            "sufficient_view": False,
            "confidence": 0.5,
            "reason": f"mock view critic requests rotation for {view_task.get('action')}",
            "adjustment": {"type": "rotate", "yaw_deg": yaw},
        }

    def _vllm_view_critic(
        self,
        view_task: dict,
        rgb_image,
        check_index: int,
        current_state: dict,
    ) -> dict:
        if rgb_image is None:
            return {
                "sufficient_view": False,
                "confidence": 0.0,
                "reason": "no RGB image has been received for view critic",
                "adjustment": None,
            }

        encoded = self._encode_rgb_image_base64(rgb_image)
        if encoded is None:
            return {
                "sufficient_view": False,
                "confidence": 0.0,
                "reason": "failed to encode RGB image for view critic",
                "adjustment": None,
            }

        payload = {
            "model": self.view_critic_vllm_model,
            "messages": [
                {
                    "role": "user",
                    "content": [
                        {"type": "text", "text": self._view_critic_prompt(view_task)},
                        {
                            "type": "image_url",
                            "image_url": {
                                "url": f"data:image/jpeg;base64,{encoded}",
                            },
                        },
                    ],
                }
            ],
            "temperature": float(self.view_critic_temperature),
            "max_tokens": int(self.view_critic_max_tokens),
        }
        try:
            request = urllib.request.Request(
                f"{self.view_critic_vllm_base_url}/chat/completions",
                data=json.dumps(payload).encode("utf-8"),
                headers={"Content-Type": "application/json"},
                method="POST",
            )
            with urllib.request.urlopen(
                request,
                timeout=float(self.view_critic_timeout_sec),
            ) as response:
                response_payload = json.loads(response.read().decode("utf-8"))
            content = response_payload["choices"][0]["message"]["content"]
            if isinstance(content, list):
                content = "".join(
                    item.get("text", "") if isinstance(item, dict) else str(item)
                    for item in content
                )
            return self._normalize_view_critic_response(
                self._parse_view_critic_json(str(content))
            )
        except Exception as exc:
            return {
                "sufficient_view": False,
                "confidence": 0.0,
                "reason": f"view critic vLLM request failed: {exc}",
                "adjustment": None,
            }

    def _view_critic_prompt(self, view_task: dict) -> str:
        action = view_task.get("action")
        target = view_task.get("target")
        destination = view_task.get("destination")
        view_target = view_task.get("view_target")
        requirement = view_task.get("requirement")
        success_condition = view_task.get("success_condition")
        return (
            "You are a robot pre-action view critic.\n\n"
            f"Action: {action}\n"
            f"Object being manipulated: {target}\n"
            f"Destination: {destination}\n"
            f"Visual target/condition to check: {view_target}\n"
            f"Requirement: {requirement}\n"
            f"Success condition: {success_condition}\n\n"
            "Decide whether the current RGB image is good enough to execute the action.\n\n"
            "Rules:\n"
            "- Match the target flexibly using visual appearance, aliases, and category; do not require exact OCR or brand text.\n"
            "- For Pringles/pringles, count a likely chips can, snack tube, or cylindrical can of chips as the target even if the logo text is not readable.\n"
            "- For grasp: pass when most of the target-like object is visible and clear enough to localize. It does not need to be perfectly centered or optimally close.\n"
            "- Fail grasp view check only when no plausible target-like object is visible, it is mostly outside the image, or it is too occluded/blurred to localize.\n"
            "- For place: pass when a reasonable empty placement surface is visible.\n"
            "- For handover: pass when a receiving human hand is visible.\n"
            f"- If not sufficient, return one simple adjustment: rotate, move_forward, or move_backward. Keep rotate within [-{abs(self.view_adjust_max_yaw_deg):.0f}, {abs(self.view_adjust_max_yaw_deg):.0f}] degrees and movement within [0.05, 0.20] m.\n"
            f"- Prefer small local rotation corrections near {self.view_adjust_scan_step_deg:.0f} degrees. If the target is not visible at all, alternate directions and expand gradually rather than making one large turn.\n"
            f"- Avoid ineffective tiny rotations below {self.view_adjust_min_yaw_deg:.0f} degrees unless no rotation is needed.\n"
            "- Return adjustment=null when sufficient_view=true.\n\n"
            "Return only valid JSON with this schema:\n"
            "{\n"
            '  "sufficient_view": true or false,\n'
            '  "confidence": 0.0,\n'
            '  "reason": "short explanation",\n'
            '  "adjustment": null or {"type": "rotate", "yaw_deg": 15.0} or {"type": "move_forward", "distance_m": 0.10} or {"type": "move_backward", "distance_m": 0.10}\n'
            "}\n"
        )

    def _parse_view_critic_json(self, text: str) -> dict:
        text = text.strip()
        if text.startswith("```"):
            text = text.strip("`").strip()
            if text.lower().startswith("json"):
                text = text[4:].strip()
        try:
            parsed = json.loads(text)
            return parsed if isinstance(parsed, dict) else {}
        except json.JSONDecodeError:
            start = text.find("{")
            end = text.rfind("}")
            if start >= 0 and end > start:
                parsed = json.loads(text[start : end + 1])
                return parsed if isinstance(parsed, dict) else {}
            raise

    def _normalize_view_critic_response(self, response: dict) -> dict:
        if not isinstance(response, dict):
            response = {}
        adjustment = response.get("adjustment")
        if not isinstance(adjustment, dict):
            adjustment = None
        sufficient_view = response.get("sufficient_view")
        if sufficient_view is None:
            sufficient_view = response.get("action_ready")

        return {
            "sufficient_view": bool(sufficient_view),
            "confidence": float(response.get("confidence", 0.0) or 0.0),
            "reason": str(response.get("reason", "")),
            "adjustment": adjustment,
        }

    def _encode_rgb_image_base64(self, rgb_image) -> str | None:
        try:
            from io import BytesIO
            from PIL import Image

            image = Image.fromarray(rgb_image, mode="RGB")
            buffer = BytesIO()
            image.save(buffer, format="JPEG", quality=90)
            return base64.b64encode(buffer.getvalue()).decode("ascii")
        except Exception as exc:
            self.get_logger().warn(f"⚠️ Failed to encode view critic image: {exc}")
            return None

    def _write_view_check_debug_image(
        self,
        view_task: dict,
        check_index: int,
        rgb_image,
        view: dict,
    ) -> str | None:
        if rgb_image is None:
            return None
        try:
            from PIL import Image, ImageDraw

            view_dir = Path(self.task_report_dir) / "view_checks"
            view_dir.mkdir(parents=True, exist_ok=True)
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
            action = str(view_task.get("action", "action"))
            view_target = str(view_task.get("view_target") or view_task.get("target") or "target")
            safe_target = "".join(
                char if char.isalnum() else "_" for char in f"{action}_{view_target}".lower()
            ).strip("_") or "target"
            path = view_dir / f"{timestamp}_{safe_target}_check{check_index}.jpg"

            image = Image.fromarray(rgb_image, mode="RGB")
            draw = ImageDraw.Draw(image)
            reason = str(view.get("reason", ""))[:100]
            adjustment = view.get("adjustment")
            overlay = (
                f"action={action} view={view_target} "
                f"sufficient={view.get('sufficient_view')} adj={adjustment}"
            )
            draw.rectangle((8, 8, min(image.width - 1, 8 + 12 * len(overlay)), 62), fill=(0, 0, 0))
            draw.text((12, 14), overlay, fill=(255, 255, 0))
            if reason:
                draw.text((12, 38), reason, fill=(255, 255, 0))
            image.save(path, format="JPEG", quality=90)
            return str(path)
        except Exception as exc:
            self.get_logger().warn(f"⚠️ Failed to write view check debug image: {exc}")
        return None

    def apply_view_adjustment_with_navigation(
        self,
        adjustment: dict,
        current_state: dict,
    ) -> dict:
        if not isinstance(adjustment, dict):
            return {
                "last_action": "view_adjustment",
                "success": False,
                "message": "view adjustment is not a dictionary",
            }

        adjustment_type = str(adjustment.get("type", "")).strip().lower()
        if adjustment_type in {"forward", "move_closer", "approach"}:
            adjustment_type = "move_forward"
        if adjustment_type in {"backward", "move_back", "back_up"}:
            adjustment_type = "move_backward"
        if adjustment_type not in {"rotate", "move_forward", "move_backward"}:
            return {
                "last_action": "view_adjustment",
                "success": False,
                "message": f"unsupported view adjustment type: {adjustment_type}",
                "adjustment": adjustment,
            }

        pose_result = self.execute_robot_pose(current_state)
        if not pose_result.get("success"):
            pose_result["last_action"] = "view_adjustment"
            return pose_result

        robot_pose = pose_result.get("result", {}).get("pose", {})
        if "x" not in robot_pose or "y" not in robot_pose or "theta" not in robot_pose:
            return {
                "last_action": "view_adjustment",
                "success": False,
                "message": "robot pose is missing x, y, or theta",
                "pose_result": pose_result,
            }

        theta = float(robot_pose["theta"])
        normalized_adjustment = {"type": adjustment_type}

        if adjustment_type == "rotate":
            try:
                yaw_deg = float(adjustment.get("yaw_deg", 0.0))
            except (TypeError, ValueError):
                yaw_deg = 0.0
            max_yaw = abs(float(self.view_adjust_max_yaw_deg))
            if max_yaw <= 0.0:
                return {
                    "last_action": "view_adjustment",
                    "success": False,
                    "message": "rotate adjustment is disabled because view_adjust_max_yaw_deg <= 0",
                    "adjustment": adjustment,
                }
            min_yaw = min(max_yaw, abs(float(self.view_adjust_min_yaw_deg)))
            if 1e-3 < abs(yaw_deg) < min_yaw:
                yaw_deg = math.copysign(min_yaw, yaw_deg)
            yaw_deg = max(-max_yaw, min(max_yaw, yaw_deg))
            adjusted_pose = {
                "x": float(robot_pose["x"]),
                "y": float(robot_pose["y"]),
                "target_theta": self._normalize_angle(theta + math.radians(yaw_deg)),
            }
            normalized_adjustment["yaw_deg"] = yaw_deg
        else:
            try:
                distance_m = float(adjustment.get("distance_m", 0.15))
            except (TypeError, ValueError):
                distance_m = 0.15
            max_forward = abs(float(self.view_adjust_max_forward_m))
            if max_forward <= 0.0:
                return {
                    "last_action": "view_adjustment",
                    "success": False,
                    "message": f"{adjustment_type} adjustment is disabled because view_adjust_max_forward_m <= 0",
                    "adjustment": adjustment,
                }
            distance_m = max(0.05, min(max_forward, abs(distance_m)))
            direction = -1.0 if adjustment_type == "move_backward" else 1.0
            adjusted_pose = {
                "x": float(robot_pose["x"]) + direction * distance_m * math.cos(theta),
                "y": float(robot_pose["y"]) + direction * distance_m * math.sin(theta),
                "target_theta": self._normalize_angle(theta),
            }
            normalized_adjustment["distance_m"] = distance_m

        self.get_logger().info(
            "👁️ Applying view adjustment with navigation: "
            + json.dumps(
                {"adjustment": normalized_adjustment, "adjusted_pose": adjusted_pose},
                ensure_ascii=False,
            )
        )
        result = self.execute_navigation(pose=adjusted_pose, current_state=current_state)
        result["last_action"] = "view_adjustment"
        result["adjustment"] = normalized_adjustment
        result["adjusted_pose"] = adjusted_pose
        return result

    def _normalize_angle(self, angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))


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

    def _execute_nav_pose(self, pose: dict, timeout_sec: float = 300.0, apply_standoff: bool = False) -> bool:
        try:
            if not self.nav_client.wait_for_server(timeout_sec=10.0):
                self.get_logger().error("❌ Nav server not available.")
                return False

            target_x = float(pose["x"])
            target_y = float(pose["y"])

            requested_theta = pose.get("target_theta", pose.get("yaw"))
            if requested_theta is not None:
                try:
                    requested_theta = float(requested_theta)
                    if not math.isfinite(requested_theta):
                        requested_theta = None
                except (TypeError, ValueError):
                    requested_theta = None

            standoff_applied = False

            if apply_standoff:
                # Goal and heading come from the occupancy map around the
                # projected target point (no bbox involved).
                (
                    goal_x,
                    goal_y,
                    goal_theta,
                    standoff_applied,
                    standoff_mode,
                    bbox_boundary,
                ) = self._make_navigation_goal(
                    target_x,
                    target_y,
                    requested_theta,
                )
            else:
                goal_x = target_x
                goal_y = target_y
                goal_theta = requested_theta

            goal = Navigate.Goal()
            goal.target_x = goal_x
            goal.target_y = goal_y
            goal.target_theta = (
                float(goal_theta)
                if goal_theta is not None
                else math.nan
            )
            goal.has_target_theta = goal_theta is not None
            if standoff_applied:
                self.get_logger().info(
                    "📏 Applying %.2f m stand-off to pose navigation: "
                    "target=(%.2f, %.2f), nav_goal=(%.2f, %.2f)"
                    % (
                        self._nav_approach_standoff_distance,
                        target_x,
                        target_y,
                        goal.target_x,
                        goal.target_y,
                    )
                )
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
