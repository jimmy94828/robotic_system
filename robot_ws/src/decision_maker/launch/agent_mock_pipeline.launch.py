import os

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="decision_maker",
                executable="agent_decision_maker_node",
                name="agent_decision_maker_node",
                output="screen",
                parameters=[
                    {
                        "enable_map_visualizer": False,
                        "mock_execution": True,
                        "mock_object_query": False,
                        "mock_failure_rate": 0.0,
                        "mock_fail_once_capabilities": "navigation",
                        "agent_max_steps": 25,
                        "agent_max_replans": 2,
                        "enable_pre_action_view_check": os.getenv("ENABLE_PRE_ACTION_VIEW_CHECK", "true").lower() == "true",
                        "view_check_max_adjustments": int(os.getenv("VIEW_CHECK_MAX_ADJUSTMENTS", "5")),
                        "view_critic_backend": os.getenv("VIEW_CRITIC_BACKEND", "mock"),
                        "view_critic_vllm_base_url": os.getenv("VIEW_CRITIC_VLLM_BASE_URL", "http://localhost:8001/v1"),
                        "view_critic_vllm_model": os.getenv("VIEW_CRITIC_VLLM_MODEL", "qwen-vl"),
                        "view_critic_image_topic": os.getenv("VIEW_CRITIC_IMAGE_TOPIC", "/camera/color/image_raw"),
                    }
                ],
            ),
        ]
    )
