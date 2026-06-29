"""Lightweight feedforward/feedback harness for agent planning.

The harness is intentionally ROS-free. It does not execute robot capabilities;
it only packages planning rules, verifies high-level calls, and records execution
observations so the closed-loop agent can replan with better context.
"""

from __future__ import annotations

import copy
from pathlib import Path
from typing import Any, Dict, List, Tuple


class LightweightPlanningHarness:
    """Small harness layer around closed-loop capability planning."""

    RULE_DOCUMENT_NAMES = (
        "DECOMPOSITION.md",
        "DECOMPOSITION_FORMAT.md",
        "PLANNING.md",
        "SKILLS.md",
        "OUTPUT_FORMAT.md",
        "VERIFICATION.md",
        "REPLANNING.md",
        "MEMORY.md",
    )

    def __init__(
        self,
        max_replans_per_task: int = 1,
        memory_limit: int = 20,
        rules_dir: str | None = None,
    ):
        self.max_replans_per_task = max(0, int(max_replans_per_task))
        self.memory_limit = max(1, int(memory_limit))
        self.rules_dir = Path(rules_dir) if rules_dir else Path(__file__).with_name("harness_rules")
        self.rule_documents = self._load_rule_documents()

    def _load_rule_documents(self) -> Dict[str, str]:
        documents = {}
        for name in self.RULE_DOCUMENT_NAMES:
            path = self.rules_dir / name
            try:
                documents[name] = path.read_text(encoding="utf-8")
            except OSError:
                documents[name] = ""
        return documents

    def build_initial_state(
        self,
        task_instruction: str,
        supported_capabilities: List[str],
        mock_execution: bool,
        extra_context: Dict[str, Any] | None = None,
    ) -> Dict[str, Any]:
        state = {
            "task": task_instruction,
            "known_poses": {},
            "held_object": None,
            "robot_pose": None,
            "step_index": 0,
            "harness": {
                "mode": "mock" if mock_execution else "live",
                "feedforward": self.build_feedforward_context(
                    supported_capabilities,
                    extra_context or {},
                ),
                "feedback": {
                    "execution_memory": [],
                    "last_failure": None,
                    "replans_used": 0,
                    "max_replans": self.max_replans_per_task,
                },
            },
        }
        return state

    def build_feedforward_context(
        self,
        supported_capabilities: List[str],
        extra_context: Dict[str, Any],
    ) -> Dict[str, Any]:
        return {
            "compact_rules": [
                "Emit exactly one next capability call, never a full plan.",
                "Use observations/history before choosing the next call.",
                "For direct grasp-only commands, call grasp_place(action=grasp) from the current view; no destination is required.",
                "For source-aware transfers, query object first; if not found, query/navigate source.",
                "After grasp in transfer tasks, query/navigate destination before place or handover.",
                "Use handover, not place, for hand over/give/pass-to-person tasks.",
                "Use finish only after the task goal is achieved.",
            ],
            "skill_constraints": {
                "object_query": "target required",
                "navigation": "target or pose required",
                "grasp_place": "action grasp/place/handover; place needs destination",
                "robot_pose": "no target",
                "finish": "task_done=true",
            },
            "allowed_capabilities": list(supported_capabilities),
            "extra_context": extra_context,
        }

    def build_decomposition_context(
        self,
        task_instruction: str,
        supported_capabilities: List[str],
        mock_execution: bool,
        extra_context: Dict[str, Any] | None = None,
    ) -> Dict[str, Any]:
        return {
            "stage": "task_decomposition",
            "original_task": task_instruction,
            "mode": "mock" if mock_execution else "live",
            "compact_rules": [
                "Split complex commands into ordered atomic subtasks.",
                "Do not emit capability calls in decomposition.",
                "Multiple objects become one subtask per object.",
                "Preserve then/and then/after that order.",
                "Preserve source and destination for from/on/in/at-source transfers.",
                "hand over/give/pass tasks become type=handover and must stay handover subtasks.",
            ],
            "output_schema": {
                "original_task": task_instruction,
                "subtasks": [
                    {
                        "subtask_id": 1,
                        "text": "handover pringles on table to sofa",
                        "type": "handover",
                        "object": "pringles",
                        "source": "table",
                        "destination": "sofa",
                    }
                ],
            },
            "supported_subtask_types": ["bring", "move", "handover", "conditional", "query", "other"],
            "supported_capabilities_after_decomposition": list(supported_capabilities),
            "extra_context": extra_context or {},
        }

    def verify_decomposition(
        self,
        original_task: str,
        decomposition: Dict[str, Any],
    ) -> Tuple[bool, str]:
        subtasks = decomposition.get("subtasks") if isinstance(decomposition, dict) else None
        if not isinstance(subtasks, list) or not subtasks:
            return False, "decomposition must contain at least one subtask."

        for expected_id, subtask in enumerate(subtasks, start=1):
            if not isinstance(subtask, dict):
                return False, f"subtask {expected_id} is not a dictionary."
            if subtask.get("subtask_id") != expected_id:
                return False, "subtask_id values must be sequential starting from 1."
            text = subtask.get("text")
            if not isinstance(text, str) or not text.strip():
                return False, f"subtask {expected_id} must contain non-empty text."

        if _looks_like_multi_object_task(original_task) and len(subtasks) == 1:
            subtask = subtasks[0]
            text = str(subtask.get("text", "")).lower()
            obj = str(subtask.get("object", "")).lower()
            if " and " in text or " and " in obj:
                return False, "multi-object command was not decomposed into separate subtasks."

        return True, "decomposition verified"

    def prepare_decision_state(
        self,
        current_state: Dict[str, Any],
        history: List[dict],
        last_execution_result: dict | None,
    ) -> Dict[str, Any]:
        harness = current_state.get("harness", {})
        feedback = harness.get("feedback", {})
        recent_history = history[-5:]
        stage = "decision"
        compact_rules = [
            "Choose one valid capability call.",
            "Use cached known_poses when available.",
            "Do not finish until the task goal is complete.",
        ]
        if not history and last_execution_result is None:
            stage = "task_start"
            compact_rules = list(harness.get("feedforward", {}).get("compact_rules", compact_rules))
        elif last_execution_result and last_execution_result.get("success") is False:
            stage = "replan"
            compact_rules = [
                "Use last_execution_result to repair the next call.",
                "If object_query(object) failed in a source-aware task, query the source.",
                "If grasp failed, retry/recover at the source; do not navigate to destination unless holding the object.",
                "Do not repeat an identical failed call without new information.",
            ]
            if last_execution_result.get("last_action") == "harness_verification":
                compact_rules.append("Respect the verifier rejection reason before choosing a new call.")

        return {
            "task": current_state.get("task"),
            "known_poses": copy.deepcopy(current_state.get("known_poses", {})),
            "held_object": current_state.get("held_object"),
            "robot_pose": current_state.get("robot_pose"),
            "step_index": current_state.get("step_index", 0),
            "harness": {
                "mode": harness.get("mode"),
                "active_context": {
                    "stage": stage,
                    "compact_rules": compact_rules,
                    "recent_memory": recent_history,
                    "last_execution_result": last_execution_result,
                    "last_failure": feedback.get("last_failure"),
                    "replans_used": feedback.get("replans_used", 0),
                    "max_replans": feedback.get("max_replans", self.max_replans_per_task),
                },
            },
        }

    def build_verification_context(
        self,
        capability_call: Dict[str, Any],
        current_state: Dict[str, Any],
        history: List[dict],
    ) -> Dict[str, Any]:
        return {
            "stage": "verification",
            "compact_rules": [
                "Schema fields must match capability requirements.",
                "Place/handover only after holding the target object and navigating to destination.",
                "After failed grasp, do not navigate to destination until grasp succeeds.",
            ],
            "candidate_call": capability_call,
            "current_state_summary": {
                "held_object": current_state.get("held_object"),
                "known_pose_targets": sorted(current_state.get("known_poses", {}).keys()),
                "robot_pose_available": bool(current_state.get("robot_pose")),
            },
            "recent_history": history[-5:],
        }

    def _select_rule_documents(self, document_names: List[str]) -> Dict[str, str]:
        return {name: self.rule_documents.get(name, "") for name in document_names}

    def verify_capability_call(
        self,
        capability_call: Dict[str, Any],
        current_state: Dict[str, Any],
        history: List[dict],
    ) -> Tuple[bool, str]:
        capability = capability_call.get("capability")
        if capability == "finish":
            if history and history[-1].get("result", {}).get("success") is False:
                return False, "finish is not allowed immediately after a failed capability."
            return True, "finish verified"

        if history:
            previous = history[-1]
            if (
                previous.get("result", {}).get("success") is False
                and previous.get("call") == capability_call
            ):
                return False, "identical failed capability call was repeated without new information."

        if capability == "navigation" and "pose" not in capability_call:
            target = capability_call.get("target")
            if target and target not in current_state.get("known_poses", {}):
                return True, "navigation target has no cached pose; executor may perform live lookup."

        if capability == "grasp_place" and capability_call.get("action") in {"place", "handover"}:
            held_object = current_state.get("held_object")
            target = capability_call.get("target")
            if not held_object:
                return False, f"cannot {capability_call.get('action')} {target}; no object is currently held."
            if target and held_object != target:
                return False, f"cannot {capability_call.get('action')} {target}; current held_object is {held_object}."

            destination = _destination_from_task(current_state.get("task", ""))
            if destination:
                current_nav = self._latest_successful_navigation_target(history)
                if current_nav != destination:
                    return (
                        False,
                        f"cannot {capability_call.get('action')} {target} before navigating to destination {destination}; "
                        f"latest navigation target is {current_nav}.",
                    )

        if capability == "navigation":
            unresolved_grasp = self._latest_unresolved_grasp_failure(history)
            if unresolved_grasp:
                last_source = self._latest_successful_navigation_target(history)
                target = capability_call.get("target")
                if target and last_source and target != last_source:
                    return (
                        False,
                        f"cannot navigate to {target} after failed grasp of {unresolved_grasp}; "
                        f"object is not held. Retry grasp or recover at source {last_source} first.",
                    )

        return True, "capability call verified"

    def _latest_unresolved_grasp_failure(self, history: List[dict]) -> str | None:
        latest_failed_target = None
        for entry in history:
            call = entry.get("call", {})
            result = entry.get("result", {})
            if call.get("capability") != "grasp_place" or call.get("action") != "grasp":
                continue
            target = call.get("target")
            if result.get("success"):
                latest_failed_target = None
            elif target:
                latest_failed_target = target
        return latest_failed_target

    def _latest_successful_navigation_target(self, history: List[dict]) -> str | None:
        for entry in reversed(history):
            call = entry.get("call", {})
            result = entry.get("result", {})
            if call.get("capability") == "navigation" and result.get("success"):
                return result.get("target") or call.get("target")
        return None

    def record_observation(
        self,
        current_state: Dict[str, Any],
        capability_call: Dict[str, Any],
        result: Dict[str, Any],
    ) -> None:
        harness = current_state.setdefault("harness", {})
        feedback = harness.setdefault("feedback", {})
        memory = feedback.setdefault("execution_memory", [])
        memory.append({"call": capability_call, "result": result})
        if len(memory) > self.memory_limit:
            del memory[:-self.memory_limit]

        if result.get("success"):
            feedback["last_failure"] = None
        else:
            feedback["last_failure"] = {"call": capability_call, "result": result}

    def can_replan_after_failure(self, current_state: Dict[str, Any]) -> bool:
        feedback = current_state.get("harness", {}).get("feedback", {})
        return int(feedback.get("replans_used", 0)) < self.max_replans_per_task

    def mark_replan_used(self, current_state: Dict[str, Any]) -> int:
        feedback = current_state.setdefault("harness", {}).setdefault("feedback", {})
        replans_used = int(feedback.get("replans_used", 0)) + 1
        feedback["replans_used"] = replans_used
        return replans_used


def _looks_like_multi_object_task(text: str) -> bool:
    normalized = f" {text.strip().lower()} "
    return " and " in normalized or "," in normalized


def _destination_from_task(text: str) -> str | None:
    normalized = str(text or "").strip().lower()
    if " to " not in normalized:
        return None
    destination = normalized.rsplit(" to ", 1)[1].strip()
    if not destination:
        return None
    destination = destination.split(" and ", 1)[0].strip(" .,;:")
    for prep in (" on ", " in ", " at ", " from ", " near ", " by ", " inside "):
        if prep in destination:
            destination = destination.split(prep, 1)[0].strip()
    for article in ("the ", "a ", "an "):
        if destination.startswith(article):
            destination = destination[len(article):].strip()
    return " ".join(destination.split()) or None
