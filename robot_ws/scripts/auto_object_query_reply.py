#!/usr/bin/env python3
"""Automatically reply to /object_query_choice for unattended mock tests."""

import json
import os
import random

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class AutoObjectQueryReply(Node):
    def __init__(self):
        super().__init__("auto_object_query_reply")
        self.reply_index = os.getenv("OBJECT_QUERY_REPLY_INDEX", "0")
        self.reply_delay_sec = float(os.getenv("OBJECT_QUERY_REPLY_DELAY_SEC", "0.3"))
        self.random_reply = os.getenv("OBJECT_QUERY_REPLY_RANDOM", "0") == "1"
        self.pub = self.create_publisher(String, "/object_query_reply", 10)
        self.sub = self.create_subscription(
            String,
            "/object_query_choice",
            self.on_choice,
            10,
        )
        self.replied_request_ids = set()
        self.get_logger().info(
            "Auto object-query reply ready "
            f"(index={self.reply_index}, random={self.random_reply})."
        )

    def on_choice(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn(f"Ignoring non-JSON /object_query_choice: {msg.data!r}")
            return

        if payload.get("event") != "active":
            return

        request_id = payload.get("request_id")
        if request_id and request_id in self.replied_request_ids:
            return

        options = payload.get("options") or []
        if not options:
            self.get_logger().warn("Ignoring object-query choice without options.")
            return

        if self.random_reply:
            index = random.choice([int(option.get("index", 0)) for option in options])
        else:
            try:
                index = int(self.reply_index)
            except ValueError:
                index = 0
            valid_indexes = {int(option.get("index", 0)) for option in options}
            if index not in valid_indexes:
                index = min(valid_indexes)

        if request_id:
            self.replied_request_ids.add(request_id)

        def publish_reply():
            reply = {"request_id": request_id, "index": index}
            out = String()
            out.data = json.dumps(reply)
            self.pub.publish(out)
            self.get_logger().info(
                f"Auto replied to object-query choice: {out.data}"
            )

        timer = self.create_timer(self.reply_delay_sec, publish_reply)

        def cancel_timer():
            timer.cancel()

        self.create_timer(self.reply_delay_sec + 0.1, cancel_timer)


def main():
    rclpy.init()
    node = AutoObjectQueryReply()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
