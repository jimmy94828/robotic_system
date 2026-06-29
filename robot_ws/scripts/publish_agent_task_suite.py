#!/usr/bin/env python3
"""Publish a suite of manual commands and wait for /task_reply after each."""

import json
import os
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


DEFAULT_TASKS = [
    "bring the pringles from the chair to the table",
    "move the bottle from the sofa to the table",
    "pick up the bottle from the table",
    "bring me the bottle from the table",
    "pick up the pringles from the chair and hand it over to me",
    "bring me the bottle from the table, I am sitting on the chair",
    "pick up the pringles from the table and hand it over to me at the sofa",
    "bring me the pringles",
    "give me the bottle",
    "hand me the object on the table",
    "bring the bottle and the pringles to the table",
    "move the bottle and the pringles from the chair to the table",
    "bring the bottle from the sofa and the pringles from the chair to the table",
    "put the bottle on the table and the pringles on the sofa",
    "bring the pringles from the chair to the table, then bring the bottle from the sofa to the table",
    "pick up the bottle from the table and hand it to me at the sofa, then bring the pringles to the table",
    "move the bottle from the sofa to the table, after that hand me the pringles from the chair",
    "I am sitting on the sofa. Please bring me the bottle from the table.",
    "Can you get the pringles from the chair and give them to me at the table?",
    "The bottle is on the sofa. Please put it on the table.",
    "I need the pringles near the table. They should be on the chair.",
    "bring me something from the table",
    "go to the table",
    "bring the bottle from the table to the sofa",
    "where are you now?",
    "what is your current position?",
    "tell me where the robot is",
]


class AgentTaskSuitePublisher(Node):
    def __init__(self):
        super().__init__("agent_task_suite_publisher")
        self.pub = self.create_publisher(String, "/manual_command", 10)
        self.sub = self.create_subscription(String, "/task_reply", self.on_reply, 10)
        self.current_task = None
        self.last_reply = None
        self.reply_event = threading.Event()

    def on_reply(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn(f"Ignoring non-JSON /task_reply: {msg.data!r}")
            return

        task = str(payload.get("task", ""))
        if self.current_task and task != self.current_task:
            self.get_logger().info(
                f"Ignoring /task_reply for another task: {task!r}"
            )
            return

        self.last_reply = payload
        self.reply_event.set()

    def publish_task(self, task: str):
        self.current_task = task
        self.last_reply = None
        self.reply_event.clear()
        msg = String()
        msg.data = task
        self.pub.publish(msg)
        self.get_logger().info(f"Published task: {task}")


def _load_tasks():
    task_file = os.getenv("TASK_SUITE_FILE", "").strip()
    if task_file:
        with open(task_file, "r", encoding="utf-8") as f:
            return [line.strip() for line in f if line.strip() and not line.startswith("#")]
    tasks_text = os.getenv("TASK_SUITE_TASKS", "").strip()
    if tasks_text:
        return [task.strip() for task in tasks_text.split("|") if task.strip()]
    return list(DEFAULT_TASKS)


def main():
    rclpy.init()
    node = AgentTaskSuitePublisher()
    tasks = _load_tasks()
    initial_delay = float(os.getenv("TASK_SUITE_INITIAL_DELAY_SEC", "12"))
    between_delay = float(os.getenv("TASK_SUITE_BETWEEN_TASKS_SEC", "3"))
    timeout_sec = float(os.getenv("TASK_SUITE_TASK_TIMEOUT_SEC", "300"))

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        node.get_logger().info(
            f"Waiting {initial_delay:.1f}s before publishing {len(tasks)} tasks."
        )
        time.sleep(initial_delay)
        results = []
        for index, task in enumerate(tasks, start=1):
            node.get_logger().info(f"=== Task {index}/{len(tasks)} ===")
            node.publish_task(task)
            if not node.reply_event.wait(timeout=timeout_sec):
                node.get_logger().error(
                    f"Timed out after {timeout_sec:.1f}s waiting for /task_reply: {task}"
                )
                results.append({"task": task, "success": False, "status": "timeout"})
            else:
                reply = node.last_reply or {}
                results.append(reply)
                node.get_logger().info(
                    "Task reply: " + json.dumps(reply, ensure_ascii=False)
                )
            time.sleep(between_delay)

        summary = {
            "total": len(results),
            "success": sum(1 for item in results if item.get("success")),
            "failed": sum(1 for item in results if not item.get("success")),
        }
        node.get_logger().info("Suite summary: " + json.dumps(summary))
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
