"""
Agent-side planning helpers for the decision_maker package.

This module is intentionally ROS-free so plan validation and conversion can be
tested without starting rclpy. Runtime execution stays in decision_maker_node.py.
"""

from __future__ import annotations

import json
import os
import re
import urllib.error
import urllib.request
from typing import Any, Dict, Iterable, List


SUPPORTED_ACTIONS = {"goto", "grasp", "place", "handover"}
SUPPORTED_CAPABILITIES = {"object_query", "navigation", "grasp_place", "robot_pose", "finish"}
SUPPORTED_GRASP_PLACE_ACTIONS = {"grasp", "place", "handover"}


class PlanValidationError(ValueError):
    """Raised when an agent plan or capability call is invalid."""


class QwenClient:
    """Modular Qwen client with placeholder, vLLM, and local Transformers backends."""

    DEFAULT_MODEL_PATH = "/home/acm/robotic_agent/models/Qwen3-8B"

    def __init__(self):
        self.backend = os.getenv("LLM_BACKEND", "local_transformers").strip().lower()
        self.model_path = os.getenv("QWEN_MODEL_PATH", self.DEFAULT_MODEL_PATH)
        self.max_new_tokens = int(os.getenv("QWEN_MAX_NEW_TOKENS", "512"))
        self.vllm_base_url = os.getenv("VLLM_BASE_URL", "http://localhost:8001").rstrip("/")
        self.vllm_model = os.getenv("VLLM_MODEL", "decision-qwen")
        self.vllm_temperature = float(os.getenv("VLLM_TEMPERATURE", "0"))
        self.vllm_timeout_sec = float(os.getenv("VLLM_TIMEOUT_SEC", "60"))
        self._model = None
        self._processor = None
        self._tokenizer = None

    def preload_model(self) -> bool:
        """Load the configured LLM backend before the first task arrives."""
        if self.backend != "local_transformers":
            return False
        self._ensure_local_model_loaded()
        return True

    # ------------------------------------------------------------------
    # Baseline single-shot plan API, kept for comparison.
    # ------------------------------------------------------------------
    def generate_plan(self, task_instruction: str, context: dict) -> dict:
        text = task_instruction.strip().lower()
        if not text:
            raise PlanValidationError("Task instruction is empty.")

        if self.backend in {"local_transformers", "vllm"}:
            return self._generate_plan_local_transformers(task_instruction, context)

        steps = self._heuristic_steps(text)
        if not steps:
            raise PlanValidationError(
                "QwenClient placeholder could not generate a plan for this task."
            )

        return {"task": task_instruction.strip(), "steps": steps}

    # ------------------------------------------------------------------
    # Closed-loop capability decision API.
    # ------------------------------------------------------------------
    def decide_next_capability(
        self,
        original_task: str,
        current_state: dict,
        history: List[dict],
        last_execution_result: dict | None,
    ) -> dict:
        text = original_task.strip().lower()
        if not text:
            raise PlanValidationError("Original task is empty.")

        if self.backend in {"local_transformers", "vllm"}:
            return self._decide_next_capability_local_transformers(
                original_task,
                current_state,
                history,
                last_execution_result,
            )

        return self._heuristic_next_capability(
            text,
            current_state,
            history,
            last_execution_result,
        )

    def _heuristic_steps(self, text: str) -> List[dict]:
        if text.startswith("go to "):
            return [{"action": "goto", "target": _clean_phrase(text[6:])}]

        if text in {"go home", "park"}:
            return [{"action": "goto", "target": "home"}]

        direct_grasp_target = _try_parse_direct_grasp_command(text)
        if direct_grasp_target:
            return [{"action": "grasp", "target": direct_grasp_target}]

        if text.startswith("give me "):
            item = _clean_phrase(text[len("give me ") :])
            return [
                {"action": "goto", "target": item},
                {"action": "grasp", "target": item},
                {"action": "goto", "target": "me"},
                {"action": "handover", "target": item},
            ]

        if text.startswith("bring me "):
            item = _clean_phrase(text[len("bring me ") :])
            return [
                {"action": "goto", "target": item},
                {"action": "grasp", "target": item},
                {"action": "goto", "target": "me"},
                {"action": "handover", "target": item},
            ]

        if text.startswith("move "):
            transfers = _try_parse_transfer_sequence(text) or []
            steps = []
            for obj, nav_target, dest, item_final_action, _fallback_source in transfers:
                final_step = {"action": item_final_action, "target": obj}
                if item_final_action == "place":
                    final_step["destination"] = dest
                steps.extend([
                    {"action": "goto", "target": nav_target},
                    {"action": "grasp", "target": obj},
                    {"action": "goto", "target": dest},
                    final_step,
                ])
            return steps

        for prefix, final_action in (
            ("bring ", "place"),
            ("place ", "place"),
            ("handover ", "handover"),
        ):
            if text.startswith(prefix):
                transfers = _parse_transfer_sequence(text[len(prefix) :], final_action)
                steps = []
                for obj, nav_target, dest, item_final_action, _fallback_source in transfers:
                    final_step = {"action": item_final_action, "target": obj}
                    if item_final_action == "place":
                        final_step["destination"] = dest
                    steps.extend([
                        {"action": "goto", "target": nav_target},
                        {"action": "grasp", "target": obj},
                        {"action": "goto", "target": dest},
                        final_step,
                    ])
                return steps

        return []

    def _heuristic_next_capability(
        self,
        text: str,
        current_state: dict,
        history: List[dict],
        last_execution_result: dict | None,
    ) -> dict:
        transfer_sequence = _try_parse_transfer_sequence(text)
        if transfer_sequence is not None:
            call = _expected_transfer_sequence_capability(
                transfer_sequence,
                len(history),
                current_state,
                last_execution_result,
            )
            if call is not None:
                return call

        if text.startswith("go to "):
            target = _clean_phrase(text[len("go to ") :])
            script = [
                {
                    "capability": "object_query",
                    "target": target,
                    "reason": "Need to locate the navigation target.",
                },
                {
                    "capability": "navigation",
                    "target": target,
                    "reason": "Navigate to the requested target.",
                },
                {
                    "capability": "finish",
                    "task_done": True,
                    "reason": "The navigation task is complete.",
                },
            ]
            return self._attach_known_pose_if_available(
                script[min(len(history), len(script) - 1)],
                current_state,
                last_execution_result,
            )

        direct_grasp_target = _try_parse_direct_grasp_command(text)
        if direct_grasp_target:
            script = [
                {
                    "capability": "grasp_place",
                    "action": "grasp",
                    "target": direct_grasp_target,
                    "reason": "Direct grasp command; attempt grasp from the current view without navigation.",
                },
                {
                    "capability": "finish",
                    "task_done": True,
                    "reason": "The direct grasp task is complete.",
                },
            ]
            return script[min(len(history), len(script) - 1)]

        if text in {"go home", "park"}:
            script = [
                {
                    "capability": "navigation",
                    "target": "home",
                    "reason": "Navigate to the home location.",
                },
                {
                    "capability": "finish",
                    "task_done": True,
                    "reason": "The robot is parked.",
                },
            ]
            return self._attach_known_pose_if_available(
                script[min(len(history), len(script) - 1)],
                current_state,
                last_execution_result,
            )

        if _is_robot_pose_query(text):
            script = [
                {
                    "capability": "robot_pose",
                    "reason": "Need to observe the robot's current pose.",
                },
                {
                    "capability": "finish",
                    "task_done": True,
                    "reason": "The robot pose has been reported.",
                },
            ]
            return script[min(len(history), len(script) - 1)]

        raise PlanValidationError(
            "QwenClient placeholder could not decide the next capability for this task."
        )

    def _attach_known_pose_if_available(
        self,
        call: dict,
        current_state: dict,
        last_execution_result: dict | None,
    ) -> dict:
        if call.get("capability") != "navigation" or "pose" in call:
            return call

        target = call.get("target")
        if not target:
            return call

        known_pose = current_state.get("known_poses", {}).get(target)
        if known_pose:
            enriched = dict(call)
            enriched["pose"] = known_pose
            return enriched

        if (
            last_execution_result
            and last_execution_result.get("last_action") == "object_query"
            and last_execution_result.get("target") == target
            and last_execution_result.get("success")
        ):
            pose = last_execution_result.get("result", {}).get("pose")
            if pose:
                enriched = dict(call)
                enriched["pose"] = pose
                return enriched

        return call

    def _decide_next_capability_local_transformers(
        self,
        original_task: str,
        current_state: dict,
        history: List[dict],
        last_execution_result: dict | None,
    ) -> dict:
        prompt = self._build_capability_prompt(
            original_task,
            current_state,
            history,
            last_execution_result,
        )
        messages = [
            {
                "role": "system",
                "content": (
                    "You are a closed-loop robot decision agent running in JSON-only mode. "
                    "Thinking mode is disabled. Never output <think> or hidden reasoning. "
                    "Return exactly one valid JSON object for the next capability call and nothing else. "
                    "The first character of your response must be { and the last character must be }."
                ),
            },
            {"role": "user", "content": prompt},
        ]
        content = self._generate_chat_content(messages)
        return self._parse_json_response(content)

    def _generate_plan_local_transformers(self, task_instruction: str, context: dict) -> dict:
        prompt = self._build_planning_prompt(task_instruction, context)
        messages = [
            {
                "role": "system",
                "content": (
                    "You are a robot task planner running in JSON-only mode. "
                    "Thinking mode is disabled. Never output <think> or hidden reasoning. "
                    "Return exactly one valid JSON object and nothing else. "
                    "The first character of your response must be { and the last character must be }."
                ),
            },
            {"role": "user", "content": prompt},
        ]
        content = self._generate_chat_content(messages)
        return self._parse_json_response(content)

    def _generate_chat_content(self, messages: List[dict]) -> str:
        if self.backend == "vllm":
            return self._generate_chat_content_vllm(messages)

        self._ensure_local_model_loaded()
        try:
            import torch
        except ImportError as exc:
            raise PlanValidationError(
                "Local Qwen backend requires torch. Install torch in the ROS container."
            ) from exc

        tokenizer_or_processor = self._processor or self._tokenizer
        if tokenizer_or_processor is None:
            raise PlanValidationError("Local Qwen backend was not loaded correctly.")

        try:
            chat_text = tokenizer_or_processor.apply_chat_template(
                messages,
                tokenize=False,
                add_generation_prompt=True,
                enable_thinking=False,
            )
        except TypeError:
            chat_text = tokenizer_or_processor.apply_chat_template(
                messages,
                tokenize=False,
                add_generation_prompt=True,
            )
        try:
            inputs = tokenizer_or_processor(text=[chat_text], return_tensors="pt")
        except TypeError:
            inputs = tokenizer_or_processor(chat_text, return_tensors="pt")

        if torch.cuda.is_available():
            inputs = inputs.to("cuda")

        with torch.inference_mode():
            generated_ids = self._model.generate(
                **inputs,
                max_new_tokens=self.max_new_tokens,
                do_sample=False,
            )

        input_len = inputs["input_ids"].shape[1]
        generated_ids = generated_ids[:, input_len:]
        return tokenizer_or_processor.batch_decode(
            generated_ids,
            skip_special_tokens=True,
            clean_up_tokenization_spaces=False,
        )[0]

    def _generate_chat_content_vllm(self, messages: List[dict]) -> str:
        payload = {
            "model": self.vllm_model,
            "messages": messages,
            "temperature": self.vllm_temperature,
            "max_tokens": self.max_new_tokens,
            "stream": False,
        }
        data = json.dumps(payload).encode("utf-8")
        request = urllib.request.Request(
            f"{self.vllm_base_url}/v1/chat/completions",
            data=data,
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        try:
            with urllib.request.urlopen(request, timeout=self.vllm_timeout_sec) as response:
                response_data = json.loads(response.read().decode("utf-8"))
        except urllib.error.HTTPError as exc:
            body = exc.read().decode("utf-8", errors="replace")
            raise PlanValidationError(
                f"vLLM request failed with HTTP {exc.code}: {body}"
            ) from exc
        except (urllib.error.URLError, TimeoutError, json.JSONDecodeError) as exc:
            raise PlanValidationError(
                f"vLLM request failed for {self.vllm_base_url}: {exc}"
            ) from exc

        choices = response_data.get("choices")
        if not choices:
            raise PlanValidationError(f"vLLM returned no choices: {response_data!r}")
        message = choices[0].get("message", {})
        content = message.get("content")
        if not isinstance(content, str) or not content.strip():
            raise PlanValidationError(f"vLLM returned empty content: {response_data!r}")
        return content

    def _ensure_local_model_loaded(self) -> None:
        if self._model is not None and (self._processor is not None or self._tokenizer is not None):
            return

        if not os.path.isdir(self.model_path):
            raise PlanValidationError(f"Local Qwen model path does not exist: {self.model_path}")

        try:
            import torch
            from transformers import AutoConfig
        except ImportError as exc:
            raise PlanValidationError(
                "Local Qwen backend requires transformers and torch. "
                "Install them in the ROS container before using LLM_BACKEND=local_transformers."
            ) from exc

        self._disable_deepgemm_hub_probe()

        dtype = torch.bfloat16 if torch.cuda.is_available() else torch.float32
        device_map = "auto" if torch.cuda.is_available() else None

        config = AutoConfig.from_pretrained(
            self.model_path,
            trust_remote_code=True,
        )
        model_type = str(getattr(config, "model_type", "")).lower()
        architectures = [str(item).lower() for item in getattr(config, "architectures", []) or []]
        is_vision_model = "vl" in model_type or any("vision" in item or "image" in item for item in architectures)

        if is_vision_model:
            try:
                from transformers import AutoProcessor
                try:
                    from transformers import AutoModelForImageTextToText as ModelClass
                except ImportError:
                    from transformers import AutoModelForVision2Seq as ModelClass
            except ImportError as exc:
                raise PlanValidationError(
                    "Installed transformers is too old for Qwen3-VL. "
                    "Upgrade transformers before loading the local vision-language model."
                ) from exc

            self._processor = AutoProcessor.from_pretrained(
                self.model_path,
                trust_remote_code=True,
            )
            self._model = ModelClass.from_pretrained(
                self.model_path,
                dtype=dtype,
                device_map=device_map,
                trust_remote_code=True,
            )
        else:
            try:
                from transformers import AutoModelForCausalLM, AutoTokenizer
            except ImportError as exc:
                raise PlanValidationError(
                    "Local text Qwen backend requires AutoTokenizer and AutoModelForCausalLM. "
                    "Upgrade transformers before loading Qwen3-8B."
                ) from exc

            self._tokenizer = AutoTokenizer.from_pretrained(
                self.model_path,
                trust_remote_code=True,
            )
            self._model = AutoModelForCausalLM.from_pretrained(
                self.model_path,
                dtype=dtype,
                device_map=device_map,
                trust_remote_code=True,
            )

        self._model.eval()

    def _disable_deepgemm_hub_probe(self) -> None:
        """Skip DeepGEMM Hub probing on non-Hopper devices.

        Jetson-class GPUs cannot use DeepGEMM, and the probe may hit Hugging
        Face rate limits. Raising ImportError here makes Transformers use the
        Triton finegrained-FP8 fallback immediately.
        """
        try:
            import transformers.integrations.finegrained_fp8 as finegrained_fp8
        except Exception:
            return

        def _unavailable_deepgemm():
            raise ImportError(
                "DeepGEMM disabled for this robot runtime; use Triton finegrained-fp8 fallback."
            )

        finegrained_fp8._load_deepgemm_kernel = _unavailable_deepgemm

    def _build_planning_prompt(self, task_instruction: str, context: dict) -> str:
        schema = {
            "task": "bring apple to table",
            "steps": [
                {"action": "goto", "target": "apple"},
                {"action": "grasp", "target": "apple"},
                {"action": "goto", "target": "table"},
                {"action": "place", "target": "apple", "destination": "table"},
            ],
        }
        payload = {
            "task_instruction": task_instruction,
            "context": context,
            "allowed_actions": sorted(SUPPORTED_ACTIONS),
            "required_json_schema_example": schema,
            "rules": [
                "Use only goto, grasp, place, and handover actions.",
                "Every step must include action and target.",
                "place steps must include destination.",
                "Use symbolic names such as apple or table, not coordinates.",
                "For '<object> on/in/at/from <source> to <destination>', goto the source, grasp only the object, then goto the destination.",
                "For multiple objects joined by and/commas, decompose into one transfer per object; never use the combined list as a target.",
                "Never use combined phrases such as 'pringle on cabinet' as a target.",
                "Return exactly one JSON object and no extra text.",
                "Do not output reasoning, markdown, comments, or <think> blocks.",
                "The response must start with { and end with }.",
            ],
        }
        return (
            json.dumps(payload, ensure_ascii=False)
            + "\n/no_think\n"
            + "Return only the JSON object. Start with { and end with }."
        )

    def _build_capability_prompt(
        self,
        original_task: str,
        current_state: dict,
        history: List[dict],
        last_execution_result: dict | None,
    ) -> str:
        payload = {
            "task": original_task,
            "state": current_state,
            "recent_history": history[-5:],
            "last_execution_result": last_execution_result,
            "allowed_capabilities": sorted(SUPPORTED_CAPABILITIES),
            "json_contract": "Return exactly one JSON object. No markdown, no reasoning, no <think>.",
            "rules": [
                "Choose one next capability call only.",
                "object_query: target required.",
                "navigation: target or pose required.",
                "grasp_place: action must be grasp/place/handover; grasp/handover need target; place needs target+destination.",
                "finish: task_done=true only after task completion.",
                "For direct commands like 'grasp the bottle', 'grab bottle', or 'pick up bottle' with no source/destination, call grasp_place action=grasp immediately from the current view; do not object_query or navigate first.",
                "For source-aware transfers, first try object_query(object); if it fails, query/navigate source.",
                "After grasp in a transfer task, query/navigate destination before final action. For direct grasp-only tasks, finish after grasp succeeds.",
                "If task says hand over/give/pass/pick up and hand over, final grasp_place action must be handover, not place.",
                "Do not navigate to destination after failed grasp unless the object is held.",
            ],
            "schemas": {
                "object_query": {"capability": "object_query", "target": "table", "reason": "..."},
                "navigation": {"capability": "navigation", "target": "table", "pose": {"x": 1.0, "y": 2.0, "theta": 0.0}, "reason": "..."},
                "grasp": {"capability": "grasp_place", "action": "grasp", "target": "pringles", "reason": "..."},
                "place": {"capability": "grasp_place", "action": "place", "target": "pringles", "destination": "sofa", "reason": "..."},
                "handover": {"capability": "grasp_place", "action": "handover", "target": "pringles", "reason": "..."},
                "finish": {"capability": "finish", "task_done": True, "reason": "..."},
            },
        }
        return (
            json.dumps(payload, ensure_ascii=False)
            + "\n/no_think\n"
            + "Return only the JSON object. Start with { and end with }."
        )

    def _parse_json_response(self, content: str) -> dict:
        text = self._strip_json_fence(self._strip_thinking_text(content))
        try:
            return json.loads(text)
        except json.JSONDecodeError:
            pass

        for candidate in self._iter_json_object_candidates(text):
            try:
                return json.loads(candidate)
            except json.JSONDecodeError:
                continue

        raise PlanValidationError(f"Local Qwen returned non-JSON content: {content!r}")

    def _strip_thinking_text(self, text: str) -> str:
        stripped = re.sub(r"<think>.*?</think>", "", text, flags=re.DOTALL | re.IGNORECASE)
        end_tag = "</think>"
        if end_tag in stripped.lower():
            index = stripped.lower().rfind(end_tag)
            stripped = stripped[index + len(end_tag):]
        return stripped.strip()

    def _iter_json_object_candidates(self, text: str):
        starts = [index for index, char in enumerate(text) if char == "{"]
        for start in starts:
            depth = 0
            in_string = False
            escaped = False
            for index in range(start, len(text)):
                char = text[index]
                if in_string:
                    if escaped:
                        escaped = False
                    elif char == "\\":
                        escaped = True
                    elif char == '"':
                        in_string = False
                    continue
                if char == '"':
                    in_string = True
                elif char == "{":
                    depth += 1
                elif char == "}":
                    depth -= 1
                    if depth == 0:
                        yield text[start:index + 1]
                        break

    def _strip_json_fence(self, text: str) -> str:
        stripped = text.strip()
        if stripped.startswith("```json"):
            stripped = stripped[len("```json") :].strip()
        elif stripped.startswith("```"):
            stripped = stripped[len("```") :].strip()
        if stripped.endswith("```"):
            stripped = stripped[:-3].strip()
        return stripped


class DecisionAgent:
    """Wrapper around an LLM client for single-shot and iterative decisions."""

    def __init__(self, llm_client: QwenClient | None = None):
        self.llm_client = llm_client or QwenClient()

    def plan(self, task_instruction: str, context: dict | None = None) -> dict:
        raw_plan = self.llm_client.generate_plan(task_instruction, context or {})
        validated = validate_plan(raw_plan)
        normalized = normalize_plan(task_instruction, validated)
        return validate_plan(normalized)

    def decide_next_capability(
        self,
        original_task: str,
        current_state: dict,
        history: List[dict],
        last_execution_result: dict | None,
    ) -> dict:
        raw_call = self.llm_client.decide_next_capability(
            original_task,
            current_state,
            history,
            last_execution_result,
        )
        try:
            validated = validate_capability_call(raw_call)
        except PlanValidationError as first_error:
            # Local LLMs sometimes choose the right high-level capability but omit
            # a required field, most commonly task_done=true for finish. For
            # structured transfer tasks, recover from the task/history-derived
            # expected next capability before failing the command.
            try:
                normalized = normalize_capability_call(
                    original_task,
                    current_state,
                    history,
                    last_execution_result,
                    raw_call if isinstance(raw_call, dict) else {},
                )
                return validate_capability_call(normalized)
            except PlanValidationError:
                raise first_error

        normalized = normalize_capability_call(
            original_task,
            current_state,
            history,
            last_execution_result,
            validated,
        )
        return validate_capability_call(normalized)


def validate_capability_call(call: Dict[str, Any]) -> Dict[str, Any]:
    """Validate and normalize a single closed-loop capability call."""
    if not isinstance(call, dict):
        raise PlanValidationError("Capability call must be a dictionary.")

    capability = _required_top_level_string(call, "capability")
    if capability not in SUPPORTED_CAPABILITIES:
        raise PlanValidationError(f"Unsupported capability '{capability}'.")

    normalized = {"capability": capability}
    reason = call.get("reason")
    if isinstance(reason, str) and reason.strip():
        normalized["reason"] = reason.strip()

    if capability == "object_query":
        normalized["target"] = _required_top_level_string(call, "target")
    elif capability == "navigation":
        target = call.get("target")
        pose = call.get("pose")
        if isinstance(target, str) and target.strip():
            normalized["target"] = target.strip().lower()
        if isinstance(pose, dict):
            normalized["pose"] = _normalize_pose(pose)
        if "target" not in normalized and "pose" not in normalized:
            raise PlanValidationError("navigation requires target or pose.")
    elif capability == "grasp_place":
        action = _required_top_level_string(call, "action")
        if action not in SUPPORTED_GRASP_PLACE_ACTIONS:
            raise PlanValidationError("grasp_place action must be 'grasp', 'place', or 'handover'.")
        normalized["action"] = action
        normalized["target"] = _required_top_level_string(call, "target")
        if action == "place":
            normalized["destination"] = _required_top_level_string(call, "destination")
        elif action == "handover":
            destination = call.get("destination")
            if isinstance(destination, str) and destination.strip():
                normalized["destination"] = destination.strip().lower()
    elif capability == "robot_pose":
        frame = call.get("frame")
        if isinstance(frame, str) and frame.strip():
            normalized["frame"] = frame.strip().lower()
    elif capability == "finish":
        if call.get("task_done") is not True:
            raise PlanValidationError("finish requires task_done=true.")
        normalized["task_done"] = True

    return normalized


def normalize_capability_call(
    original_task: str,
    current_state: dict,
    history: List[dict],
    last_execution_result: dict | None,
    call: Dict[str, Any],
) -> Dict[str, Any]:
    """Ground next capability calls for common transfer commands."""
    direct_grasp_target = _try_parse_direct_grasp_command(original_task)
    if direct_grasp_target:
        script = [
            {
                "capability": "grasp_place",
                "action": "grasp",
                "target": direct_grasp_target,
                "reason": "Direct grasp command; attempt grasp from the current view without navigation.",
            },
            {
                "capability": "finish",
                "task_done": True,
                "reason": "The direct grasp task is complete.",
            },
        ]
        expected = dict(script[min(len(history), len(script) - 1)])
        if call.get("capability") == expected.get("capability") and call.get("reason"):
            expected["reason"] = call["reason"]
        return expected

    transfer_sequence = _try_parse_transfer_sequence(original_task)
    if transfer_sequence is None:
        return call

    # Multi-object transfer decomposition is intentionally left to the LLM.
    # The harness feeds PLANNING.md/SKILLS.md/OUTPUT_FORMAT.md at task start so
    # the model can reason through each object. We keep deterministic grounding
    # only for single-object transfer safety and for the placeholder backend.
    if _is_multi_object_transfer_sequence(transfer_sequence):
        return call

    expected = _expected_transfer_sequence_capability(
        transfer_sequence,
        len(history),
        current_state,
        last_execution_result,
    )
    if expected is None:
        return call

    # Safety for transfer tasks: after grasp, destination lookup/navigation must
    # happen before place/handover. This prevents an LLM from handing over while
    # the robot is still at the source location.
    if call.get("capability") == expected.get("capability") and call.get("reason"):
        expected = dict(expected)
        expected["reason"] = call["reason"]
    return expected


def _expected_transfer_capability(
    obj: str,
    nav_target: str,
    dest: str,
    final_action: str,
    step_index: int,
    current_state: dict,
    last_execution_result: dict | None,
) -> dict | None:
    return _expected_transfer_sequence_capability(
        [(obj, nav_target, dest, final_action, None)],
        step_index,
        current_state,
        last_execution_result,
    )


def _expected_transfer_sequence_capability(
    transfers: List[tuple[str, str, str, str, str | None]],
    step_index: int,
    current_state: dict,
    last_execution_result: dict | None,
) -> dict | None:
    script = []
    for obj, primary_target, dest, final_action, fallback_source in transfers:
        source_target = _resolve_source_target(
            obj,
            primary_target,
            fallback_source,
            current_state,
            last_execution_result,
        )
        script.extend([
            {"capability": "object_query", "target": source_target, "reason": f"Need to locate the source for {obj}."},
            {"capability": "navigation", "target": source_target, "reason": f"Navigate to the source for {obj}."},
            {"capability": "grasp_place", "action": "grasp", "target": obj, "reason": f"Grasp {obj} at the source."},
            {"capability": "object_query", "target": dest, "reason": f"Need to locate the destination before placing {obj}."},
            {"capability": "navigation", "target": dest, "reason": f"Navigate to the destination for {obj}."},
        ])
        if final_action == "place":
            script.append(
                {"capability": "grasp_place", "action": "place", "target": obj, "destination": dest, "reason": f"Place {obj} at the destination."}
            )
        elif final_action == "handover":
            script.append(
                {"capability": "grasp_place", "action": "handover", "target": obj, "reason": f"Hand over {obj} at the destination."}
            )
    script.append({"capability": "finish", "task_done": True, "reason": "The transfer task is complete."})

    call = dict(script[min(step_index, len(script) - 1)])
    if call.get("capability") == "navigation":
        pose = current_state.get("known_poses", {}).get(call.get("target"))
        if not pose and last_execution_result:
            if (
                last_execution_result.get("last_action") == "object_query"
                and last_execution_result.get("target") == call.get("target")
                and last_execution_result.get("success")
            ):
                pose = last_execution_result.get("result", {}).get("pose")
        if pose:
            call["pose"] = pose
    return call


def _resolve_source_target(
    obj: str,
    primary_target: str,
    fallback_source: str | None,
    current_state: dict,
    last_execution_result: dict | None,
) -> str:
    known_poses = current_state.get("known_poses", {})
    if primary_target in known_poses:
        return primary_target

    if fallback_source:
        if fallback_source in known_poses:
            return fallback_source
        if last_execution_result and last_execution_result.get("last_action") == "object_query":
            last_target = last_execution_result.get("target")
            if last_target == primary_target and last_execution_result.get("success") is False:
                return fallback_source
            if last_target == fallback_source and last_execution_result.get("success"):
                return fallback_source

    return primary_target


def normalize_plan(task_instruction: str, plan: Dict[str, Any]) -> Dict[str, Any]:
    """Apply deterministic safety normalization to an LLM-produced plan."""
    transfer_sequence = _try_parse_transfer_sequence(task_instruction)
    if transfer_sequence is None:
        return plan

    # Let the LLM own multi-object decomposition in single-shot comparisons.
    # Placeholder plans are already decomposed before this normalization step.
    if _is_multi_object_transfer_sequence(transfer_sequence):
        return plan

    steps = []
    for obj, nav_target, dest, final_action, _fallback_source in transfer_sequence:
        final_step = {"action": final_action, "target": obj}
        if final_action == "place":
            final_step["destination"] = dest
        steps.extend([
            {"action": "goto", "target": nav_target},
            {"action": "grasp", "target": obj},
            {"action": "goto", "target": dest},
            final_step,
        ])

    return {
        "task": plan.get("task") or task_instruction.strip(),
        "steps": steps,
    }


def validate_plan(plan: Dict[str, Any]) -> Dict[str, Any]:
    """Validate and normalize a structured JSON-like task plan."""
    if not isinstance(plan, dict):
        raise PlanValidationError("Plan must be a dictionary.")

    task = plan.get("task")
    if not isinstance(task, str) or not task.strip():
        raise PlanValidationError("Plan must include a non-empty string 'task'.")

    steps = plan.get("steps")
    if not isinstance(steps, list) or not steps:
        raise PlanValidationError("Plan must include a non-empty list 'steps'.")

    normalized_steps = []
    for index, step in enumerate(steps):
        if not isinstance(step, dict):
            raise PlanValidationError(f"Step {index} must be a dictionary.")

        action = _required_string(step, "action", index).lower()
        if action not in SUPPORTED_ACTIONS:
            raise PlanValidationError(f"Step {index} has unsupported action '{action}'.")

        target = _required_string(step, "target", index)
        normalized = {"action": action, "target": target}

        if action == "place":
            normalized["destination"] = _required_string(step, "destination", index)
        elif "destination" in step and isinstance(step["destination"], str):
            destination = step["destination"].strip().lower()
            if destination:
                normalized["destination"] = destination

        normalized_steps.append(normalized)

    return {"task": task.strip(), "steps": normalized_steps}


def plan_to_primitives(plan: Dict[str, Any]) -> List[str]:
    """Convert a validated plan to legacy decision_maker primitive strings."""
    validated = validate_plan(plan)
    primitives = []

    for step in validated["steps"]:
        action = step["action"]
        target = step["target"]
        if action == "goto":
            primitives.append(f"goto:{target}")
        elif action == "grasp":
            primitives.append(f"grasp:{target}")
        elif action == "place":
            primitives.append(f"place:{target}:{step['destination']}")
        elif action == "handover":
            primitives.append(f"handover:{target}")
        else:
            raise PlanValidationError(f"Unsupported action '{action}'.")

    return primitives


def _required_top_level_string(data: Dict[str, Any], key: str) -> str:
    value = data.get(key)
    if not isinstance(value, str) or not value.strip():
        raise PlanValidationError(f"Capability call must include a non-empty '{key}'.")
    return value.strip().lower()


def _normalize_pose(pose: Dict[str, Any]) -> Dict[str, float]:
    normalized = {}
    for key in ("x", "y", "z", "theta"):
        if key in pose:
            try:
                normalized[key] = float(pose[key])
            except (TypeError, ValueError) as exc:
                raise PlanValidationError(f"pose.{key} must be numeric.") from exc
    if "x" not in normalized or "y" not in normalized:
        raise PlanValidationError("pose requires at least numeric x and y.")
    return normalized


def _required_string(step: Dict[str, Any], key: str, index: int) -> str:
    value = step.get(key)
    if not isinstance(value, str) or not value.strip():
        raise PlanValidationError(f"Step {index} must include a non-empty '{key}'.")
    return value.strip().lower()


_PREPOSITIONS = r"(?:on|in|at|near|by|inside|from|above|below|under|beside|next\s+to)"


def _try_parse_direct_grasp_command(task_instruction: str) -> str | None:
    text = task_instruction.strip().lower()
    text = re.sub(r"[.!?]+$", "", text).strip()
    match = re.match(r"^(?:please\s+)?(?:grasp|grab|pick\s+up|pickup|take|hold)\s+(.+)$", text)
    if not match:
        return None
    target = _clean_phrase(match.group(1))
    if not target:
        return None
    # Source/destination phrases imply a larger navigation or transfer task, not
    # a pure current-view grasp. Let the transfer planner handle those.
    if re.search(r"\b(?:from|on|in|at|to|onto|into|near|by|inside)\b", target):
        return None
    return target


def _is_robot_pose_query(text: str) -> bool:
    normalized = text.strip().lower()
    english_patterns = (
        "where are you",
        "where is the robot",
        "where is robot",
        "current robot pose",
        "current robot position",
        "robot current pose",
        "robot current position",
        "get robot pose",
        "get current pose",
    )
    if any(pattern in normalized for pattern in english_patterns):
        return True

    chinese_patterns = (
        "現在位置",
        "目前位置",
        "機器人位置",
        "機器人在哪",
        "你在哪",
    )
    return any(pattern in normalized for pattern in chinese_patterns)


def _is_multi_object_transfer_sequence(transfers: List[tuple[str, str, str, str, str | None]]) -> bool:
    return len(transfers) > 1


def _try_parse_transfer_command(task_instruction: str) -> tuple[str, str, str, str, str | None] | None:
    sequence = _try_parse_transfer_sequence(task_instruction)
    if not sequence:
        return None
    return sequence[0]


def _try_parse_transfer_sequence(task_instruction: str) -> List[tuple[str, str, str, str, str | None]] | None:
    text = task_instruction.strip().lower()
    pickup_handover = re.match(
        r"^(?:pick up|get|grab)\s+(.+?)\s+(?:from|on|in|at)\s+(.+?)\s+and\s+(?:hand\s+it\s+over|handover\s+it|give\s+it|pass\s+it)\s+to\s+(.+)$",
        text,
    )
    if pickup_handover:
        obj, source, destination = pickup_handover.groups()
        obj = _clean_phrase(obj)
        source = _clean_phrase(source)
        destination = _clean_handover_destination(destination)
        if obj and source and destination:
            return [(obj, obj, destination, "handover", source)]

    if text.startswith("move "):
        match = re.match(r"^move\s+(.+?)\s+from\s+(.+?)\s+to\s+(.+)$", text)
        if not match:
            return None
        object_phrase, source, destination = match.groups()
        destination = _clean_phrase(re.split(r"\s+" + _PREPOSITIONS + r"\s+", destination)[0])
        source = _clean_phrase(source)
        objects = _split_object_list(object_phrase)
        if not objects or not source or not destination:
            return None
        return [(obj, obj, destination, "place", source) for obj in objects]

    for prefix, final_action in (
        ("bring ", "place"),
        ("place ", "place"),
        ("handover ", "handover"),
    ):
        if text.startswith(prefix):
            try:
                return _parse_transfer_sequence(text[len(prefix) :], final_action)
            except PlanValidationError:
                return None
    return None


def _clean_handover_destination(text: str) -> str:
    cleaned = _clean_phrase(re.split(r"\s+" + _PREPOSITIONS + r"\s+", text.strip().lower())[0])
    match = re.search(r"\b(?:me|person|user|human)\s+(?:at|on|in|near|by)\s+(.+)$", text.strip().lower())
    if match:
        return _clean_phrase(match.group(1))
    return cleaned

def _parse_transfer(arg: str) -> tuple[str, str, str]:
    first = _parse_transfer_sequence(arg, "place")[0]
    return first[:3]


def _parse_transfer_sequence(arg: str, final_action: str) -> List[tuple[str, str, str, str, str | None]]:
    text = arg.strip().lower()
    text = re.sub(r"^\s*from\s+", "", text)

    if " to " in text:
        src, dest = [part.strip() for part in text.split(" to ", 1)]
    elif " into " in text:
        src, dest = [part.strip() for part in text.split(" into ", 1)]
    else:
        parts = text.split()
        if len(parts) < 2:
            raise PlanValidationError(f"Cannot parse transfer task: '{arg}'.")
        src, dest = parts[0], parts[1]

    src = _clean_phrase(src)
    dest = _clean_phrase(re.split(r"\s+" + _PREPOSITIONS + r"\s+", dest)[0])

    prep_match = re.search(r"\s+" + _PREPOSITIONS + r"\s+", src)
    if prep_match:
        obj_phrase = _clean_phrase(src[: prep_match.start()])
        shared_nav_target = _clean_phrase(src[prep_match.end() :])
    else:
        obj_phrase = src
        shared_nav_target = None

    objects = _split_object_list(obj_phrase)
    if not objects or not dest:
        raise PlanValidationError(f"Cannot parse transfer task: '{arg}'.")

    transfers = []
    for obj in objects:
        primary_target = obj
        fallback_source = shared_nav_target
        if not obj or not primary_target:
            raise PlanValidationError(f"Cannot parse transfer task: '{arg}'.")
        transfers.append((obj, primary_target, dest, final_action, fallback_source))
    return transfers


def _split_object_list(text: str) -> List[str]:
    normalized = re.sub(r"\s*,\s*and\s+", ", ", text.strip().lower())
    parts = re.split(r"\s*(?:,|\band\b)\s*", normalized)
    return [_clean_phrase(part) for part in parts if _clean_phrase(part)]


def _clean_phrase(text: str) -> str:
    cleaned = re.sub(r"^(the|a|an)\s+", "", text.strip().lower())
    return " ".join(cleaned.split())


def primitives_from_steps(steps: Iterable[Dict[str, Any]]) -> List[str]:
    """Convenience helper for tests and future callers that only have steps."""
    return plan_to_primitives({"task": "ad hoc", "steps": list(steps)})
