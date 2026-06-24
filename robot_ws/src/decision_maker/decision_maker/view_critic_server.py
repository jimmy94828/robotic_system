"""HTTP view-critic wrapper for a vLLM-hosted VLM.

This small server exposes the custom `/view_critic` endpoint expected by
AgentDecisionMakingNode and forwards each request to a vLLM OpenAI-compatible
chat-completions server.
"""

from __future__ import annotations

import argparse
import json
import os
import re
import urllib.error
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any


DEFAULT_VLLM_BASE_URL = "http://localhost:8001/v1"
DEFAULT_MODEL = "/home/acm/robotic_agent/models/Qwen3-VL-8B-Instruct"


def _json_response(handler: BaseHTTPRequestHandler, status: int, payload: dict) -> None:
    data = json.dumps(payload, ensure_ascii=False).encode("utf-8")
    handler.send_response(status)
    handler.send_header("Content-Type", "application/json; charset=utf-8")
    handler.send_header("Content-Length", str(len(data)))
    handler.end_headers()
    handler.wfile.write(data)


def _extract_json_object(text: str) -> dict:
    text = text.strip()
    if text.startswith("```"):
        text = re.sub(r"^```(?:json)?", "", text, flags=re.IGNORECASE).strip()
        text = re.sub(r"```$", "", text).strip()
    try:
        parsed = json.loads(text)
        if isinstance(parsed, dict):
            return parsed
    except json.JSONDecodeError:
        pass

    match = re.search(r"\{.*\}", text, flags=re.DOTALL)
    if not match:
        raise ValueError("VLM response did not contain a JSON object")
    parsed = json.loads(match.group(0))
    if not isinstance(parsed, dict):
        raise ValueError("VLM JSON response was not an object")
    return parsed


def _normalize_view_response(payload: dict[str, Any]) -> dict[str, Any]:
    adjustment = payload.get("adjustment")
    if not isinstance(adjustment, dict):
        adjustment = None

    target_visible = payload.get("target_visible")
    if target_visible is None:
        target_visible = payload.get("view_target_visible")
    if target_visible is None:
        target_visible = payload.get("condition_visible")

    return {
        "target_visible": bool(target_visible),
        "sufficient_view": bool(payload.get("sufficient_view", False)),
        "confidence": float(payload.get("confidence", 0.0) or 0.0),
        "reason": str(payload.get("reason", "")),
        "adjustment": adjustment,
    }


def _build_prompt(payload: dict[str, Any]) -> str:
    action = payload.get("action", "unknown")
    target = payload.get("target", "")
    destination = payload.get("destination")
    view_target = payload.get("view_target") or target
    requirement = payload.get("requirement", "")
    success_condition = payload.get("success_condition", "")

    return f"""You are a robot pre-action view critic.

Action: {action}
Object being manipulated: {target}
Destination: {destination}
Visual target/condition to check: {view_target}
Requirement: {requirement}
Success condition: {success_condition}

Decide whether the current RGB image satisfies the success condition.

Rules:
- For grasp: the target object must be visible and sufficiently clear/centered for grasping.
- For place: an empty, reachable, flat placement area at or near the destination must be visible.
- For handover: a human hand must be visible and positioned to receive the object.
- If the view is not sufficient, suggest a robot-relative rotation.
- Use yaw_deg > 0 for turning left and yaw_deg < 0 for turning right.
- Keep yaw_deg within [-45, 45].
- If no useful adjustment is possible, set adjustment to null.

Return only valid JSON with this schema:
{{
  "target_visible": true or false,
  "sufficient_view": true or false,
  "confidence": 0.0,
  "reason": "short explanation",
  "adjustment": null or {{"type": "rotate", "yaw_deg": -25.0}}
}}
"""


class ViewCriticHandler(BaseHTTPRequestHandler):
    server_version = "ViewCriticServer/0.1"

    def do_GET(self) -> None:
        if self.path == "/health":
            _json_response(self, 200, {"ok": True})
            return
        _json_response(self, 404, {"error": "not found"})

    def do_POST(self) -> None:
        if self.path != "/view_critic":
            _json_response(self, 404, {"error": "not found"})
            return

        try:
            length = int(self.headers.get("Content-Length", "0"))
            payload = json.loads(self.rfile.read(length).decode("utf-8"))
            result = self.server.view_critic(payload)  # type: ignore[attr-defined]
            _json_response(self, 200, result)
        except Exception as exc:
            _json_response(
                self,
                500,
                {
                    "target_visible": False,
                    "sufficient_view": False,
                    "confidence": 0.0,
                    "reason": f"view critic server error: {exc}",
                    "adjustment": None,
                },
            )

    def log_message(self, fmt: str, *args: Any) -> None:
        print(f"[view_critic] {self.address_string()} - {fmt % args}")


class ViewCriticServer(ThreadingHTTPServer):
    def __init__(
        self,
        server_address: tuple[str, int],
        handler_class: type[BaseHTTPRequestHandler],
        *,
        vllm_base_url: str,
        model: str,
        timeout_sec: float,
        max_tokens: int,
        temperature: float,
    ) -> None:
        super().__init__(server_address, handler_class)
        self.vllm_base_url = vllm_base_url.rstrip("/")
        self.model = model
        self.timeout_sec = timeout_sec
        self.max_tokens = max_tokens
        self.temperature = temperature

    def view_critic(self, payload: dict[str, Any]) -> dict[str, Any]:
        image_base64 = payload.get("image_base64")
        if not image_base64:
            return {
                "target_visible": False,
                "sufficient_view": False,
                "confidence": 0.0,
                "reason": "request did not include image_base64",
                "adjustment": None,
            }

        prompt = _build_prompt(payload)
        request_payload = {
            "model": self.model,
            "messages": [
                {
                    "role": "user",
                    "content": [
                        {"type": "text", "text": prompt},
                        {
                            "type": "image_url",
                            "image_url": {
                                "url": f"data:image/jpeg;base64,{image_base64}",
                            },
                        },
                    ],
                }
            ],
            "temperature": self.temperature,
            "max_tokens": self.max_tokens,
        }
        request = urllib.request.Request(
            f"{self.vllm_base_url}/chat/completions",
            data=json.dumps(request_payload).encode("utf-8"),
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        with urllib.request.urlopen(request, timeout=self.timeout_sec) as response:
            response_payload = json.loads(response.read().decode("utf-8"))

        content = response_payload["choices"][0]["message"]["content"]
        if isinstance(content, list):
            content = "".join(
                item.get("text", "") if isinstance(item, dict) else str(item)
                for item in content
            )
        return _normalize_view_response(_extract_json_object(str(content)))


def main() -> None:
    parser = argparse.ArgumentParser(description="View critic wrapper for vLLM VLMs.")
    parser.add_argument("--host", default=os.getenv("VIEW_CRITIC_HOST", "0.0.0.0"))
    parser.add_argument("--port", type=int, default=int(os.getenv("VIEW_CRITIC_PORT", "8002")))
    parser.add_argument("--vllm-base-url", default=os.getenv("VLLM_BASE_URL", DEFAULT_VLLM_BASE_URL))
    parser.add_argument("--model", default=os.getenv("VIEW_CRITIC_MODEL", DEFAULT_MODEL))
    parser.add_argument("--timeout-sec", type=float, default=float(os.getenv("VIEW_CRITIC_TIMEOUT_SEC", "60")))
    parser.add_argument("--max-tokens", type=int, default=int(os.getenv("VIEW_CRITIC_MAX_TOKENS", "256")))
    parser.add_argument("--temperature", type=float, default=float(os.getenv("VIEW_CRITIC_TEMPERATURE", "0")))
    args = parser.parse_args()

    server = ViewCriticServer(
        (args.host, args.port),
        ViewCriticHandler,
        vllm_base_url=args.vllm_base_url,
        model=args.model,
        timeout_sec=args.timeout_sec,
        max_tokens=args.max_tokens,
        temperature=args.temperature,
    )
    print(
        f"[view_critic] serving on http://{args.host}:{args.port}/view_critic; "
        f"vLLM={args.vllm_base_url}, model={args.model}"
    )
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
