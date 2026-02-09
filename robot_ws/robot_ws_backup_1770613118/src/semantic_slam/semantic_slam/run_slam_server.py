import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse
from semantic_slam_interfaces.action import RunSlam
import numpy as np
import os, time
from urllib.parse import urlparse
from datetime import datetime

class RunSlamServer(Node):
    def __init__(self):
        super().__init__('run_slam_server')
        self._server = ActionServer(
            self, RunSlam, 'run_slam',
            execute_callback=self.execute_cb,
            cancel_callback=self.cancel_cb
        )
        self.map_in_memory = None  # save map
        self.get_logger().info('RunSlam action server ready on /run_slam')

    def cancel_cb(self, goal_handle):
        self.get_logger().info('Cancel requested.')
        return CancelResponse.ACCEPT

    def _resolve_output_uri(self, base_uri: str) -> str:
        # "file:///tmp/gs_map" -> "file:///tmp/gs_map_YYYY-mm-dd-HH-MM-SS.npz"
        ts = datetime.now().strftime('%Y-%m-%d-%H-%M-%S')
        if not base_uri:
            base_uri = 'file:///tmp/gs_map'
        return f"{base_uri}_{ts}.npz"

    def _save_npz(self, map_data: dict, uri: str) -> str:
        p = urlparse(uri)
        assert p.scheme in ('file', ''), f'Only file:// supported currently: {uri}'
        path = p.path if p.scheme == 'file' else uri
        os.makedirs(os.path.dirname(path), exist_ok=True)
        np.savez_compressed(path, **map_data)
        return f'file://{path}'

    def _fake_increment(self, step: int) -> dict:
        # fake SLAM increment data
        n = 10000  # num of splats / step
        means3D = np.random.uniform(-3, 3, (n, 3)).astype(np.float32)
        unnorm_rot = np.tile(np.array([0,0,0,1], np.float32), (n,1))
        log_scales = np.full((n,1), -3.0, np.float32)
        logit_opacities = np.full((n,1), 2.0, np.float32)
        rgb_colors = np.clip(np.random.rand(n,3), 0, 1).astype(np.float32)
        semantic_ids = np.random.randint(0, 20, size=(n,), dtype=np.uint8)
        semantic_colors = np.clip(np.random.rand(n,3), 0, 1).astype(np.float32)
        return dict(
            means3D=means3D, # (N,3)
            unnorm_rotations=unnorm_rot,
            log_scales=log_scales,
            logit_opacities=logit_opacities,
            rgb_colors=rgb_colors,
            semantic_ids=semantic_ids, # (N,1)
            semantic_colors=semantic_colors, # (N,3)
        )

    async def execute_cb(self, goal_handle):
        req = goal_handle.request
        self.get_logger().info(f"RunSlam start: save_on_stop={req.save_on_stop}, out_prefix={req.output_uri}")

        # self.map_in_memory = None
        total_steps = 30
        num_splats = 0

        # SLAM Simulation
        for step in range(total_steps):
            if goal_handle.is_cancel_requested:
                self.get_logger().info('RunSlam canceled by client.')
                map_uri = ''
                if req.save_on_stop and self.map_in_memory is not None:
                    out = self._resolve_output_uri(req.output_uri)
                    map_uri = self._save_npz(self.map_in_memory, out)
                goal_handle.canceled()
                result = RunSlam.Result(success=True, map_uri=map_uri, num_splats=int(num_splats))
                return result

            inc = self._fake_increment(step)
            if self.map_in_memory is None:
                self.map_in_memory = inc
            else:
                for k in self.map_in_memory:
                    self.map_in_memory[k] = np.concatenate([self.map_in_memory[k], inc[k]], axis=0)
            num_splats = int(self.map_in_memory['means3D'].shape[0])

            # feedback
            fb = RunSlam.Feedback()
            fb.num_splats = num_splats
            goal_handle.publish_feedback(fb)

            # fake processing time
            start = time.time()
            while time.time() - start < 0.1:
                rclpy.spin_once(self, timeout_sec=0.01)

        # Finish
        map_uri = ''
        if req.save_on_stop and self.map_in_memory is not None:
            out = self._resolve_output_uri(req.output_uri)
            map_uri = self._save_npz(self.map_in_memory, out)

        goal_handle.succeed()
        result = RunSlam.Result(success=True, map_uri=map_uri, num_splats=num_splats)
        self.get_logger().info(f'RunSlam finished. saved={bool(map_uri)} num_splats={num_splats}')
        return result

def main():
    rclpy.init()
    node = RunSlamServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
