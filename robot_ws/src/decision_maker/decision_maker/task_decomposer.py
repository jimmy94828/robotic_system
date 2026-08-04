"""Task decomposition helpers for agent-based decision making.

This module is ROS-free. It decomposes a complex user command into simpler
atomic subtasks before each subtask is handed to the existing closed-loop agent
planner/executor.
"""

from __future__ import annotations

import json
import re
from typing import Any, Dict, List

from .decision_agent import (
    PlanValidationError,
    QwenClient,
    parse_direct_grasp_place_command,
    parse_pickup_handover_command,
    parse_source_grasp_place_command,
)


SUPPORTED_SUBTASK_TYPES = {
    "bring", "move", "handover", "direct_grasp_place", "conditional", "query", "other"
}


class TaskDecompositionError(ValueError):
    """Raised when task decomposition output is invalid."""


class DecomposerClient:
    """Modular decomposer backend with local-Qwen and deterministic fallback."""

    def __init__(self, llm_client: QwenClient | None = None):
        self.llm_client = llm_client or QwenClient()

    def decompose_task(self, task_instruction: str, context: dict | None = None) -> dict:
        text = task_instruction.strip()
        if not text:
            raise TaskDecompositionError("Task instruction is empty.")

        if self.llm_client.backend in {"local_transformers", "vllm"}:
            return self._decompose_task_local_transformers(text, context or {})

        return self._decompose_task_placeholder(text)

    def _decompose_task_local_transformers(self, task_instruction: str, context: dict) -> dict:
        # Reuse the existing QwenClient generation path to avoid adding a
        # second model/API lifecycle.
        if self.llm_client.backend == "local_transformers":
            self.llm_client._ensure_local_model_loaded()
        prompt = self._build_decomposition_prompt(task_instruction, context)
        messages = [
            {
                "role": "system",
                "content": (
                    "You are a robot task decomposer running in JSON-only mode. "
                    "Thinking mode is disabled. Never output <think> or hidden reasoning. "
                    "Return exactly one valid JSON object and nothing else. "
                    "The first character of your response must be { and the last character must be }."
                ),
            },
            {"role": "user", "content": prompt},
        ]
        content = self.llm_client._generate_chat_content(messages)
        return self.llm_client._parse_json_response(content)

    def _build_decomposition_prompt(self, task_instruction: str, context: dict) -> str:
        payload = {
            "task_instruction": task_instruction,
            "goal": "Return ordered atomic subtasks for the robot agent.",
            "context": {
                "stage": context.get("stage"),
                "mode": context.get("mode"),
                "compact_rules": context.get("compact_rules", []),
                "output_schema": context.get("output_schema"),
            },
            "schema": {
                "original_task": task_instruction,
                "subtasks": [
                    {
                        "subtask_id": 1,
                        "text": "bring pringles on table to chair",
                        "type": "bring",
                        "object": "pringles",
                        "source": "table",
                        "destination": "chair",
                    }
                ],
            },
            "rules": [
                "JSON only; start with { and end with }.",
                "No reasoning, markdown, comments, or <think> blocks.",
                "Do not output object_query/navigation/grasp_place/finish here.",
                "Multiple objects joined by and/commas become separate ordered subtasks.",
                "Preserve source and destination for from/on/in/at-source transfers.",
                "A transfer to a place, surface, or furniture is type=bring/move and ends with place; never change it to handover.",
                "Use type=handover only when the command explicitly says handover/hand over/give/pass, or explicitly identifies a human receiver.",
                "Words such as bring/take/carry followed only by a destination mean grasp-and-place, not handover.",
                "For 'grasp X and place it on Y' with no source, use type=direct_grasp_place; the robot is already in front of X.",
                "For 'grasp X on/from S and place it on Y', preserve S and use type=bring; the robot must navigate to S before grasping.",
                "For 'pick up X from/on S and hand it over to me at D', output one handover subtask: handover X on S to D.",
                "For 'grasp X from S and hand it to the person sitting on D', output handover X on S to D; never use person as destination.",
                "For then/and then/after that, preserve order.",
                "Resolve it/them to the mentioned object when clear.",
            ],
            "examples": [
                {
                    "input": "bring bottle and apple to table",
                    "subtasks": [
                        {"subtask_id": 1, "text": "bring bottle to table", "type": "bring", "object": "bottle", "source": None, "destination": "table"},
                        {"subtask_id": 2, "text": "bring apple to table", "type": "bring", "object": "apple", "source": None, "destination": "table"},
                    ],
                },
                {
                    "input": "pick up the pringle from the table and hand it over to me at the sofa",
                    "subtasks": [
                        {"subtask_id": 1, "text": "handover pringle on table to sofa", "type": "handover", "object": "pringle", "source": "table", "destination": "sofa"}
                    ],
                },
            ],
        }
        return (
            json.dumps(payload, ensure_ascii=False)
            + "\n/no_think\n"
            + "Return only the JSON object. Start with { and end with }."
        )

    def _decompose_task_placeholder(self, task_instruction: str) -> dict:
        clauses = _split_sequential_clauses(task_instruction)
        subtasks = []
        last_object = None

        for clause in clauses:
            clause = _clean_text(clause)
            if not clause:
                continue

            conditional = _parse_conditional_clause(clause, last_object)
            if conditional:
                subtasks.append(conditional)
                last_object = conditional.get("object") or last_object
                continue

            parsed = _parse_transfer_clause(clause, last_object)
            if parsed:
                for subtask in parsed:
                    subtasks.append(subtask)
                    last_object = subtask.get("object") or last_object
                continue

            subtasks.append({
                "text": clause,
                "type": "other",
                "object": None,
                "source": None,
                "destination": None,
            })

        if not subtasks:
            subtasks = [{
                "text": _clean_text(task_instruction),
                "type": "other",
                "object": None,
                "source": None,
                "destination": None,
            }]

        return _with_ids({"original_task": task_instruction.strip(), "subtasks": subtasks})


class TaskDecomposer:
    """Validating wrapper around the decomposer backend."""

    def __init__(self, client: DecomposerClient | None = None):
        self.client = client or DecomposerClient()

    def decompose(self, task_instruction: str, context: dict | None = None) -> dict:
        raw = self.client.decompose_task(task_instruction, context or {})
        return validate_decomposition(task_instruction, raw)

    def decompose_texts(self, task_instruction: str, context: dict | None = None) -> List[str]:
        plan = self.decompose(task_instruction, context)
        return [subtask["text"] for subtask in plan["subtasks"]]


def validate_decomposition(original_task: str, decomposition: Dict[str, Any]) -> Dict[str, Any]:
    if not isinstance(decomposition, dict):
        raise TaskDecompositionError("Decomposition must be a dictionary.")

    task = decomposition.get("original_task")
    if not isinstance(task, str) or not task.strip():
        task = original_task.strip()

    subtasks = decomposition.get("subtasks")
    if not isinstance(subtasks, list) or not subtasks:
        raise TaskDecompositionError("Decomposition must include a non-empty subtasks list.")

    pickup_handover = parse_pickup_handover_command(original_task)
    if pickup_handover:
        obj, source, destination = pickup_handover
        return {
            "original_task": task.strip(),
            "subtasks": [
                {
                    "subtask_id": 1,
                    "text": f"handover {obj} on {source} to {destination}",
                    "type": "handover",
                    "object": obj,
                    "source": source,
                    "destination": destination,
                }
            ],
        }

    source_grasp_place = parse_source_grasp_place_command(original_task)
    if source_grasp_place:
        obj, source, destination = source_grasp_place
        return {
            "original_task": task.strip(),
            "subtasks": [
                {
                    "subtask_id": 1,
                    "text": f"bring {obj} on {source} to {destination}",
                    "type": "bring",
                    "object": obj,
                    "source": source,
                    "destination": destination,
                }
            ],
        }

    direct_transfer = parse_direct_grasp_place_command(original_task)
    if direct_transfer:
        obj, destination = direct_transfer
        return {
            "original_task": task.strip(),
            "subtasks": [
                {
                    "subtask_id": 1,
                    "text": f"grasp {obj} and place it on {destination}",
                    "type": "direct_grasp_place",
                    "object": obj,
                    "source": None,
                    "destination": destination,
                }
            ],
        }

    normalized = []
    for index, subtask in enumerate(subtasks, start=1):
        if not isinstance(subtask, dict):
            raise TaskDecompositionError(f"Subtask {index} must be a dictionary.")
        text = subtask.get("text")
        if not isinstance(text, str) or not text.strip():
            raise TaskDecompositionError(f"Subtask {index} must include non-empty text.")

        subtask_type = subtask.get("type")
        if not isinstance(subtask_type, str) or not subtask_type.strip():
            subtask_type = "other"
        subtask_type = subtask_type.strip().lower()
        if subtask_type not in SUPPORTED_SUBTASK_TYPES:
            subtask_type = "other"

        normalized_subtask = {
            "subtask_id": int(subtask.get("subtask_id") or index),
            "text": _clean_text(text),
            "type": subtask_type,
            "object": _optional_clean(subtask.get("object")),
            "source": _optional_clean(subtask.get("source")),
            "destination": _optional_clean(subtask.get("destination")),
        }

        # Handover changes the physical behavior, so do not trust an inferred
        # handover unless the user's own instruction requested one.
        if (
            normalized_subtask["type"] == "handover"
            or _starts_with_handover(normalized_subtask["text"])
        ) and not requests_handover(original_task):
            normalized_subtask = _normalize_as_place_transfer(
                original_task, normalized_subtask
            )

        normalized.append(normalized_subtask)

    for index, subtask in enumerate(normalized, start=1):
        subtask["subtask_id"] = index

    return {"original_task": task.strip(), "subtasks": normalized}


def requests_handover(task_instruction: str) -> bool:
    """Return whether the user explicitly requested a human handover."""
    text = _clean_text(task_instruction)
    if re.search(
        r"\b(?:handover|hand\s+(?:(?:it|them|the\s+\w+)\s+)?over|"
        r"hand\s+(?:it|them|the\s+\w+)\s+to|give|pass)\b",
        text,
        flags=re.IGNORECASE,
    ):
        return True

    return re.search(
        r"\bto\s+(?:me|him|her|them|us|someone|somebody|"
        r"(?:the\s+)?(?:person|human|user|operator|customer|guest|recipient|man|woman))"
        r"(?=\s*(?:$|[,.!?]|\s+(?:at|on|in|near|by)\b))",
        text,
        flags=re.IGNORECASE,
    ) is not None


def is_human_query_target(target: str) -> bool:
    """Return whether a semantic query target denotes a person, not a place."""
    normalized = _clean_text(target)
    return re.match(
        r"^(?:me|him|her|them|us|someone|somebody|person|human|user|"
        r"operator|customer|guest|recipient|man|woman)(?:\b|$)",
        normalized,
        flags=re.IGNORECASE,
    ) is not None


def _starts_with_handover(text: str) -> bool:
    return re.match(
        r"^(?:handover|hand\s+over|give|pass)\b", text, re.IGNORECASE
    ) is not None


def _normalize_as_place_transfer(
    original_task: str,
    subtask: Dict[str, Any],
) -> Dict[str, Any]:
    normalized = dict(subtask)
    original = _clean_text(original_task)
    transfer_type = "move" if re.match(r"^move\b", original, re.IGNORECASE) else "bring"
    normalized["type"] = transfer_type

    obj = normalized.get("object")
    source = normalized.get("source")
    destination = normalized.get("destination")
    if obj and destination:
        if transfer_type == "move" and source:
            normalized["text"] = f"move {obj} from {source} to {destination}"
        else:
            text = f"bring {obj}"
            if source:
                text += f" on {source}"
            normalized["text"] = f"{text} to {destination}"
    else:
        normalized["text"] = re.sub(
            r"^(?:handover|hand\s+over|give|pass)\b",
            "bring",
            normalized["text"],
            count=1,
            flags=re.IGNORECASE,
        )
    return normalized


def _with_ids(decomposition: Dict[str, Any]) -> Dict[str, Any]:
    for index, subtask in enumerate(decomposition["subtasks"], start=1):
        subtask["subtask_id"] = index
    return decomposition


def _split_sequential_clauses(text: str) -> List[str]:
    normalized = text.strip()
    normalized = re.sub(r"\s+and\s+then\s+", " then ", normalized, flags=re.IGNORECASE)
    normalized = re.sub(r"\s+after\s+that\s+", " then ", normalized, flags=re.IGNORECASE)
    normalized = re.sub(r"\s*,\s*then\s+", " then ", normalized, flags=re.IGNORECASE)
    return [part.strip(" ,") for part in re.split(r"\s+then\s+", normalized, flags=re.IGNORECASE) if part.strip(" ,")]


def _parse_conditional_clause(clause: str, last_object: str | None) -> dict | None:
    text = _clean_text(clause)
    match = re.match(
        r"^if\s+(?:the\s+)?(.+?)\s+is\s+(?:on|in|at|inside)\s+(?:the\s+)?(.+?),\s*(.+)$",
        text,
        flags=re.IGNORECASE,
    )
    if not match:
        return None

    obj = _clean_text(match.group(1))
    source = _clean_text(match.group(2))
    action = _clean_text(match.group(3))
    action = _replace_pronoun_object(action, obj or last_object)

    transfer = _parse_transfer_clause(action, obj)
    destination = transfer[0].get("destination") if transfer else None
    normalized_action = transfer[0]["text"] if transfer else action
    return {
        "text": f"if {obj} is on {source}, {normalized_action}",
        "type": "conditional",
        "object": obj,
        "source": source,
        "destination": destination,
    }


def _parse_transfer_clause(clause: str, last_object: str | None) -> List[dict] | None:
    text = _replace_pronoun_object(_clean_text(clause), last_object)

    pickup_handover = re.match(
        r"^(?:pick up|get|grab)\s+(.+?)\s+(?:from|on|in|at)\s+(.+?)\s+and\s+(?:hand\s+it\s+over|handover\s+it|give\s+it|pass\s+it)\s+to\s+(.+)$",
        text,
        flags=re.IGNORECASE,
    )
    if pickup_handover:
        obj, source, destination = pickup_handover.groups()
        obj = _clean_text(obj)
        source = _clean_text(source)
        destination = _strip_handover_destination(destination)
        return [{
            "text": f"handover {obj} on {source} to {destination}",
            "type": "handover",
            "object": obj,
            "source": source,
            "destination": destination,
        }]

    for pattern, task_type in (
        (r"^(?:handover|hand\s+over|give|pass)\s+(.+?)\s+to\s+(.+)$", "handover"),
        (r"^(?:bring|take|carry)\s+(.+?)\s+to\s+(.+)$", "bring"),
        (r"^move\s+(.+?)\s+from\s+(.+?)\s+to\s+(.+)$", "move"),
    ):
        match = re.match(pattern, text, flags=re.IGNORECASE)
        if not match:
            continue

        if task_type == "move":
            object_phrase, source, destination = match.groups()
        else:
            object_phrase, destination = match.groups()
            source = None

        if task_type == "bring" and requests_handover(text):
            task_type = "handover"

        destination = _strip_handover_destination(destination) if task_type == "handover" else _strip_destination_modifiers(destination)
        objects = _split_object_list(object_phrase)
        if not objects:
            return None

        subtasks = []
        for obj in objects:
            source_for_obj = source
            nav_obj = obj
            prep_match = re.search(r"\s+(?:on|in|at|from|inside)\s+", obj)
            if prep_match:
                nav_obj = _clean_text(obj[: prep_match.start()])
                source_for_obj = _clean_text(obj[prep_match.end():])

            if task_type == "move":
                subtask_text = f"move {nav_obj} from {_clean_text(source_for_obj)} to {destination}"
            elif task_type == "handover":
                subtask_text = f"handover {nav_obj}"
                if source_for_obj:
                    subtask_text += f" on {_clean_text(source_for_obj)}"
                subtask_text += f" to {destination}"
            else:
                subtask_text = f"bring {nav_obj}"
                if source_for_obj:
                    subtask_text += f" on {_clean_text(source_for_obj)}"
                subtask_text += f" to {destination}"
            subtasks.append({
                "text": subtask_text,
                "type": task_type,
                "object": nav_obj,
                "source": _optional_clean(source_for_obj),
                "destination": destination,
            })
        return subtasks
    return None


def _strip_handover_destination(text: str) -> str:
    cleaned = _strip_destination_modifiers(text)
    match = re.search(r"\b(?:me|person|user|human)\s+(?:at|on|in|near|by)\s+(.+)$", cleaned)
    if match:
        return _clean_text(match.group(1))
    return cleaned

def _split_object_list(text: str) -> List[str]:
    normalized = _clean_text(text)
    normalized = re.sub(r"\s*,\s*and\s+", ", ", normalized, flags=re.IGNORECASE)
    parts = re.split(r"\s*(?:,|\band\b)\s*", normalized, flags=re.IGNORECASE)
    return [_clean_text(part) for part in parts if _clean_text(part)]


def _replace_pronoun_object(text: str, obj: str | None) -> str:
    if not obj:
        return text
    return re.sub(r"\b(it|them)\b", obj, text, flags=re.IGNORECASE)


def _strip_destination_modifiers(text: str) -> str:
    cleaned = _clean_text(text)
    return re.split(r"\s+(?:and|then|after that)\s+", cleaned, maxsplit=1, flags=re.IGNORECASE)[0]


def _optional_clean(value: Any) -> str | None:
    if not isinstance(value, str):
        return None
    cleaned = _clean_text(value)
    return cleaned or None


def _clean_text(text: str) -> str:
    cleaned = re.sub(r"^(the|a|an)\s+", "", str(text).strip().lower())
    return " ".join(cleaned.split())
