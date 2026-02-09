#!/usr/bin/env python3
"""
3D Map to 2D LiDAR Map Alignment System
========================================

完整的地圖對齊系統，將 Uni3R/3DGS 等 3D 地圖與 Nav2 的 2D LiDAR 地圖對齊

"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from tf2_ros import Buffer, TransformListener, StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
import yaml
import json
from pathlib import Path
from typing import List, Dict, Tuple, Optional
import logging

# ============================================================================
# 數學工具函數
# ============================================================================

def quaternion_to_rotation_matrix(q: np.ndarray) -> np.ndarray:
    """
    四元數轉旋轉矩陣
    
    Args:
        q: [qx, qy, qz, qw] 或 [qw, qx, qy, qz]
    
    Returns:
        3x3 旋轉矩陣
    """
    # 自動檢測格式（假設 |w| 最大的是 qw）
    if abs(q[0]) < abs(q[3]):  # [qx, qy, qz, qw]
        qx, qy, qz, qw = q
    else:  # [qw, qx, qy, qz]
        qw, qx, qy, qz = q
    
    return Rotation.from_quat([qx, qy, qz, qw]).as_matrix()


def rotation_matrix_to_quaternion(R: np.ndarray) -> np.ndarray:
    """
    旋轉矩陣轉四元數
    
    Returns:
        [qx, qy, qz, qw]
    """
    return Rotation.from_matrix(R).as_quat()


def transform_to_matrix(trans: TransformStamped) -> np.ndarray:
    """
    ROS TransformStamped 轉 4x4 矩陣
    """
    T = np.eye(4)
    
    # 平移
    T[0, 3] = trans.transform.translation.x
    T[1, 3] = trans.transform.translation.y
    T[2, 3] = trans.transform.translation.z
    
    # 旋轉
    q = [
        trans.transform.rotation.x,
        trans.transform.rotation.y,
        trans.transform.rotation.z,
        trans.transform.rotation.w
    ]
    T[:3, :3] = quaternion_to_rotation_matrix(q)
    
    return T


def matrix_to_transform(T: np.ndarray, parent_frame: str, child_frame: str, 
                       timestamp) -> TransformStamped:
    """
    4x4 矩陣轉 ROS TransformStamped
    """
    trans = TransformStamped()
    trans.header.stamp = timestamp
    trans.header.frame_id = parent_frame
    trans.child_frame_id = child_frame
    
    # 平移
    trans.transform.translation.x = float(T[0, 3])
    trans.transform.translation.y = float(T[1, 3])
    trans.transform.translation.z = float(T[2, 3])
    
    # 旋轉
    q = rotation_matrix_to_quaternion(T[:3, :3])
    trans.transform.rotation.x = float(q[0])
    trans.transform.rotation.y = float(q[1])
    trans.transform.rotation.z = float(q[2])
    trans.transform.rotation.w = float(q[3])
    
    return trans


def extract_yaw(T: np.ndarray) -> float:
    """從變換矩陣提取 yaw 角"""
    return np.arctan2(T[1, 0], T[0, 0])


def params_to_transform(params: np.ndarray) -> np.ndarray:
    """
    6 參數向量轉 4x4 變換矩陣
    
    Args:
        params: [tx, ty, tz, rx, ry, rz] (平移 + 旋轉角)
    
    Returns:
        4x4 變換矩陣
    """
    T = np.eye(4)
    T[:3, 3] = params[:3]  # 平移
    T[:3, :3] = Rotation.from_rotvec(params[3:6]).as_matrix()  # 旋轉
    return T


def transform_to_params(T: np.ndarray) -> np.ndarray:
    """
    4x4 變換矩陣轉 6 參數向量
    
    Returns:
        [tx, ty, tz, rx, ry, rz]
    """
    params = np.zeros(6)
    params[:3] = T[:3, 3]  # 平移
    params[3:6] = Rotation.from_matrix(T[:3, :3]).as_rotvec()  # 旋轉
    return params


def rotation_error(R1: np.ndarray, R2: np.ndarray) -> np.ndarray:
    """
    計算兩個旋轉矩陣之間的誤差（軸角表示）
    
    Returns:
        3D 向量，表示旋轉誤差
    """
    R_error = R1.T @ R2
    return Rotation.from_matrix(R_error).as_rotvec()


# ============================================================================
# 資料載入與處理
# ============================================================================

class DataLoader:
    """資料載入器"""
    
    @staticmethod
    def load_uni3r_poses(file_path: str) -> Tuple[np.ndarray, List[np.ndarray]]:
        """
        載入 Uni3R 位姿檔案
        
        支援格式:
        1. timestamp tx ty tz qx qy qz qw
        2. 只有 tx ty tz qx qy qz qw (無時間戳)
        Returns:
            timestamps: Nx1 時間戳陣列 (若無則用索引)
            poses: List of 4x4 變換矩陣
        """
        data = np.loadtxt(file_path)
        
        # 判斷格式
        if data.shape[1] == 8:  # 有時間戳
            timestamps = data[:, 0]
            pose_data = data[:, 1:]
        elif data.shape[1] == 7:  # 無時間戳
            timestamps = np.arange(len(data), dtype=float)
            pose_data = data
        else:
            raise ValueError(f"不支援的格式: {data.shape[1]} 列")
        
        poses = []
        for row in pose_data:
            T = np.eye(4)
            T[:3, 3] = row[:3]  # tx, ty, tz
            T[:3, :3] = quaternion_to_rotation_matrix(row[3:7])  # qx,qy,qz,qw
            poses.append(T)
        
        return timestamps, poses
    
    @staticmethod
    def load_calibration(file_path: str) -> np.ndarray:
        """
        載入相機-底盤外參
        支援格式:
        - YAML: {translation: [x,y,z], rotation: [qx,qy,qz,qw]}
        - JSON: 同上
        - TXT: 4x4 矩陣
        """
        path = Path(file_path)
        
        if path.suffix in ['.yaml', '.yml']:
            with open(file_path, 'r') as f:
                data = yaml.safe_load(f)
            
            T = np.eye(4)
            T[:3, 3] = np.array(data['translation'])
            T[:3, :3] = quaternion_to_rotation_matrix(np.array(data['rotation']))
            return T
        
        elif path.suffix == '.json':
            with open(file_path, 'r') as f:
                data = json.load(f)
            
            T = np.eye(4)
            T[:3, 3] = np.array(data['translation'])
            T[:3, :3] = quaternion_to_rotation_matrix(np.array(data['rotation']))
            return T
        
        elif path.suffix == '.txt':
            return np.loadtxt(file_path)
        
        else:
            raise ValueError(f"不支援的檔案格式: {path.suffix}")


# ============================================================================
# 軌跡同步
# ============================================================================

class TrajectorySync:
    """軌跡時間同步"""
    
    @staticmethod
    def synchronize(timestamps_2d: np.ndarray, poses_2d: List[np.ndarray],
                   timestamps_3d: np.ndarray, poses_3d: List[np.ndarray],
                   max_time_diff: float = 0.05) -> List[Dict]:
        """
        同步兩條軌跡
        
        Args:
            timestamps_2d: 2D 軌跡時間戳
            poses_2d: 2D 位姿列表
            timestamps_3d: 3D 軌跡時間戳
            poses_3d: 3D 位姿列表
            max_time_diff: 最大時間差容忍度 (秒)
        
        Returns:
            同步後的配對列表
        """
        synced_pairs = []
        
        for i, (t2d, pose2d) in enumerate(zip(timestamps_2d, poses_2d)):
            # 找最近的 3D 時間戳
            time_diffs = np.abs(timestamps_3d - t2d)
            idx_3d = np.argmin(time_diffs)
            time_diff = time_diffs[idx_3d]
            
            if time_diff < max_time_diff:
                synced_pairs.append({
                    'index_2d': i,
                    'index_3d': idx_3d,
                    'time_2d': t2d,
                    'time_3d': timestamps_3d[idx_3d],
                    'pose_2d': pose2d,
                    'pose_3d': poses_3d[idx_3d],
                    'time_diff': time_diff
                })
        
        return synced_pairs
    
    @staticmethod
    def report_sync_quality(synced_pairs: List[Dict]):
        """報告同步品質"""
        if not synced_pairs:
            return "⚠️ 警告: 沒有成功同步的位姿對！"
        
        time_diffs = [p['time_diff'] for p in synced_pairs]
        
        report = f"""
同步品質報告:
============
總配對數: {len(synced_pairs)}
平均時間差: {np.mean(time_diffs)*1000:.2f} ms
最大時間差: {np.max(time_diffs)*1000:.2f} ms
標準差: {np.std(time_diffs)*1000:.2f} ms

建議:
"""
        if np.max(time_diffs) > 0.05:
            report += "⚠️ 警告: 存在 >50ms 的時間差，可能影響對齊精度\n"
        if len(synced_pairs) < 50:
            report += "⚠️ 警告: 配對數過少，建議收集更多資料\n"
        if np.mean(time_diffs) < 0.01:
            report += "✓ 時間同步品質良好\n"
        
        return report


# ============================================================================
# 座標轉換
# ============================================================================

class CoordinateTransform:
    """座標系轉換"""
    
    @staticmethod
    def camera_pose_to_base_pose(T_map3d_camera: np.ndarray,
                                 T_base_camera: np.ndarray) -> np.ndarray:
        """
        將相機位姿轉為底盤位姿
        
        Args:
            T_map3d_camera: 相機在 3D map 下的位姿
            T_base_camera: 相機相對底盤的外參
        
        Returns:
            底盤在 3D map 下的位姿
        """
        T_camera_base = np.linalg.inv(T_base_camera)
        return T_map3d_camera @ T_camera_base
    
    @staticmethod
    def transform_trajectory(poses_camera: List[np.ndarray],
                           T_base_camera: np.ndarray) -> List[np.ndarray]:
        """批量轉換軌跡"""
        return [
            CoordinateTransform.camera_pose_to_base_pose(T_cam, T_base_camera)
            for T_cam in poses_camera
        ]


# ============================================================================
# 對齊算法
# ============================================================================

class MapAlignment:
    """地圖對齊算法"""
    
    @staticmethod
    def estimate_2d_alignment(poses_2d: List[np.ndarray],
                             poses_3d: List[np.ndarray]) -> Tuple[np.ndarray, np.ndarray]:
        """
        2D 平面對齊 (3 DoF: x, y, yaw)
        
        Returns:
            R: 2x2 旋轉矩陣
            t: 2D 平移向量
        """
        # 提取 xy 位置
        points_2d = np.array([[T[0, 3], T[1, 3]] for T in poses_2d])  # Nx2
        points_3d = np.array([[T[0, 3], T[1, 3]] for T in poses_3d])  # Nx2
        
        # 計算質心
        centroid_2d = points_2d.mean(axis=0)
        centroid_3d = points_3d.mean(axis=0)
        
        # 去中心化
        points_2d_centered = points_2d - centroid_2d
        points_3d_centered = points_3d - centroid_3d
        
        # SVD 求旋轉
        H = points_3d_centered.T @ points_2d_centered
        U, S, Vt = np.linalg.svd(H)
        R = Vt.T @ U.T
        
        # 確保 det(R) = 1
        if np.linalg.det(R) < 0:
            Vt[-1, :] *= -1
            R = Vt.T @ U.T
        
        # 計算平移
        t = centroid_2d - R @ centroid_3d
        
        return R, t
    
    @staticmethod
    def refine_3d_alignment(poses_2d: List[np.ndarray],
                           poses_3d: List[np.ndarray],
                           T_init: np.ndarray,
                           loss_type: str = 'huber') -> np.ndarray:
        """
        3D 對齊精化 (6 DoF)
        
        Args:
            poses_2d: 2D 地圖下的位姿
            poses_3d: 3D 地圖下的位姿
            T_init: 初始變換
            loss_type: 損失函數類型 ('linear', 'huber', 'soft_l1')
        
        Returns:
            精化後的 4x4 變換矩陣
        """
        def residual_function(params):
            T = params_to_transform(params)
            
            residuals = []
            for p2d, p3d in zip(poses_2d, poses_3d):
                # 預測的 2D 位置
                p2d_pred = T @ p3d
                
                # 位置誤差 (權重較高)
                pos_error = (p2d[:3, 3] - p2d_pred[:3, 3]) * 10.0
                
                # 旋轉誤差 (權重較低)
                rot_error = rotation_error(p2d[:3, :3], p2d_pred[:3, :3])
                
                residuals.extend([*pos_error, *rot_error])
            
            return np.array(residuals)
        
        # 初始猜測
        x0 = transform_to_params(T_init)
        
        # 優化
        result = least_squares(
            residual_function,
            x0,
            method='lm',  # Levenberg-Marquardt
            loss=loss_type,
            verbose=0
        )
        
        return params_to_transform(result.x)
    
    @staticmethod
    def robust_alignment(poses_2d: List[np.ndarray],
                        poses_3d: List[np.ndarray],
                        ransac_iterations: int = 1000,
                        inlier_threshold: float = 0.1) -> Tuple[np.ndarray, np.ndarray]:
        """
        使用 RANSAC 的 robust 對齊
        
        Args:
            poses_2d: 2D 位姿列表
            poses_3d: 3D 位姿列表
            ransac_iterations: RANSAC 迭代次數
            inlier_threshold: 內點閾值 (公尺)
        
        Returns:
            T_best: 最佳變換矩陣
            inliers: 內點遮罩
        """
        n_poses = len(poses_2d)
        best_inliers = np.zeros(n_poses, dtype=bool)
        best_T = np.eye(4)
        best_score = 0
        
        for iteration in range(ransac_iterations):
            # 隨機選 5 個點
            if n_poses < 5:
                indices = np.arange(n_poses)
            else:
                indices = np.random.choice(n_poses, 5, replace=False)
            
            sample_2d = [poses_2d[i] for i in indices]
            sample_3d = [poses_3d[i] for i in indices]
            
            try:
                # 估計變換
                R_2d, t_2d = MapAlignment.estimate_2d_alignment(sample_2d, sample_3d)
                
                # 升到 3D
                T = np.eye(4)
                T[:2, :2] = R_2d
                T[:2, 3] = t_2d
                
                # 計算所有點的誤差
                errors = []
                for p2d, p3d in zip(poses_2d, poses_3d):
                    pred = T @ p3d
                    error = np.linalg.norm(p2d[:3, 3] - pred[:3, 3])
                    errors.append(error)
                
                # 統計內點
                inliers = np.array(errors) < inlier_threshold
                score = inliers.sum()
                
                if score > best_score:
                    best_score = score
                    best_inliers = inliers
                    best_T = T
            
            except:
                continue
        
        # 用所有內點重新估計
        if best_score > 0:
            inlier_poses_2d = [poses_2d[i] for i in np.where(best_inliers)[0]]
            inlier_poses_3d = [poses_3d[i] for i in np.where(best_inliers)[0]]
            
            # 先 2D 對齊
            R_2d, t_2d = MapAlignment.estimate_2d_alignment(inlier_poses_2d, inlier_poses_3d)
            T_init = np.eye(4)
            T_init[:2, :2] = R_2d
            T_init[:2, 3] = t_2d
            
            # 再 3D 精化
            final_T = MapAlignment.refine_3d_alignment(
                inlier_poses_2d,
                inlier_poses_3d,
                T_init
            )
            
            return final_T, best_inliers
        else:
            return best_T, best_inliers


# ============================================================================
# 軌跡品質檢查
# ============================================================================

class TrajectoryQualityChecker:
    """軌跡品質檢查器"""
    
    @staticmethod
    def check_trajectory_rotation(poses: List[np.ndarray]) -> Dict:
        """檢查軌跡旋轉豐富度"""
        yaw_changes = []
        for i in range(1, len(poses)):
            yaw1 = extract_yaw(poses[i-1])
            yaw2 = extract_yaw(poses[i])
            yaw_changes.append(abs(yaw2 - yaw1))
        
        total_rotation = sum(yaw_changes)
        
        return {
            'total_rotation_deg': np.degrees(total_rotation),
            'avg_rotation_deg': np.degrees(np.mean(yaw_changes)),
            'max_rotation_deg': np.degrees(np.max(yaw_changes)),
            'sufficient': total_rotation > np.radians(90)  # 至少 90 度
        }
    
    @staticmethod
    def check_trajectory_coverage(poses: List[np.ndarray]) -> Dict:
        """檢查軌跡覆蓋範圍"""
        positions = np.array([[T[0, 3], T[1, 3]] for T in poses])
        
        x_range = positions[:, 0].max() - positions[:, 0].min()
        y_range = positions[:, 1].max() - positions[:, 1].min()
        
        return {
            'x_range': x_range,
            'y_range': y_range,
            'area': x_range * y_range,
            'sufficient': x_range > 2.0 and y_range > 2.0  # 至少 2x2 公尺
        }
    
    @staticmethod
    def generate_report(poses_2d: List[np.ndarray],
                       poses_3d: List[np.ndarray]) -> str:
        """生成品質報告"""
        rot_2d = TrajectoryQualityChecker.check_trajectory_rotation(poses_2d)
        rot_3d = TrajectoryQualityChecker.check_trajectory_rotation(poses_3d)
        cov_2d = TrajectoryQualityChecker.check_trajectory_coverage(poses_2d)
        cov_3d = TrajectoryQualityChecker.check_trajectory_coverage(poses_3d)
        
        report = f"""
軌跡品質報告:
============

2D 軌跡:
  位姿數量: {len(poses_2d)}
  總旋轉量: {rot_2d['total_rotation_deg']:.1f}°
  覆蓋範圍: {cov_2d['x_range']:.2f}m × {cov_2d['y_range']:.2f}m
  狀態: {'✓ 足夠' if rot_2d['sufficient'] and cov_2d['sufficient'] else '⚠️ 不足'}

3D 軌跡:
  位姿數量: {len(poses_3d)}
  總旋轉量: {rot_3d['total_rotation_deg']:.1f}°
  覆蓋範圍: {cov_3d['x_range']:.2f}m × {cov_3d['y_range']:.2f}m
  狀態: {'✓ 足夠' if rot_3d['sufficient'] and cov_3d['sufficient'] else '⚠️ 不足'}

建議:
"""
        if not rot_2d['sufficient']:
            report += "⚠️ 2D 軌跡旋轉不足，建議讓機器人多轉彎\n"
        if not rot_3d['sufficient']:
            report += "⚠️ 3D 軌跡旋轉不足，可能影響 yaw 估計精度\n"
        if not cov_2d['sufficient']:
            report += "⚠️ 2D 軌跡覆蓋範圍過小，建議擴大移動範圍\n"
        if not cov_3d['sufficient']:
            report += "⚠️ 3D 軌跡覆蓋範圍過小，建議擴大移動範圍\n"
        
        return report


# ============================================================================
# 對齊誤差計算
# ============================================================================

class AlignmentError:
    """對齊誤差計算"""
    
    @staticmethod
    def compute_error(T: np.ndarray,
                     poses_2d: List[np.ndarray],
                     poses_3d: List[np.ndarray]) -> Dict:
        """
        計算對齊誤差
        
        Returns:
            誤差統計資訊
        """
        position_errors = []
        rotation_errors = []
        
        for p2d, p3d in zip(poses_2d, poses_3d):
            # 預測位置
            p2d_pred = T @ p3d
            
            # 位置誤差
            pos_err = np.linalg.norm(p2d[:3, 3] - p2d_pred[:3, 3])
            position_errors.append(pos_err)
            
            # 旋轉誤差
            rot_err = np.linalg.norm(rotation_error(p2d[:3, :3], p2d_pred[:3, :3]))
            rotation_errors.append(rot_err)
        
        return {
            'position_mean': np.mean(position_errors),
            'position_std': np.std(position_errors),
            'position_max': np.max(position_errors),
            'rotation_mean_deg': np.degrees(np.mean(rotation_errors)),
            'rotation_std_deg': np.degrees(np.std(rotation_errors)),
            'rotation_max_deg': np.degrees(np.max(rotation_errors)),
            'n_poses': len(poses_2d)
        }
    
    @staticmethod
    def generate_report(error_dict: Dict) -> str:
        """生成誤差報告"""
        return f"""
對齊誤差報告:
============
位置誤差:
  平均: {error_dict['position_mean']*100:.2f} cm
  標準差: {error_dict['position_std']*100:.2f} cm
  最大: {error_dict['position_max']*100:.2f} cm

旋轉誤差:
  平均: {error_dict['rotation_mean_deg']:.2f}°
  標準差: {error_dict['rotation_std_deg']:.2f}°
  最大: {error_dict['rotation_max_deg']:.2f}°

評估:
  {'✓ 對齊品質優秀' if error_dict['position_mean'] < 0.05 else ''}
  {'⚠️ 對齊品質一般' if 0.05 <= error_dict['position_mean'] < 0.15 else ''}
  {'❌ 對齊品質較差，建議重新對齊' if error_dict['position_mean'] >= 0.15 else ''}
"""


# ============================================================================
# ROS 2 整合節點
# ============================================================================

class MapAlignmentNode(Node):
    """
    地圖對齊 ROS 2 節點
    
    功能:
    1. 收集 2D SLAM 位姿
    2. 載入 3D SLAM 位姿
    3. 執行對齊
    4. 發布靜態 TF
    """
    
    def __init__(self):
        super().__init__('map_alignment_node')
        
        # 設置日誌
        self.logger = self.get_logger()
        self.logger.info('地圖對齊節點啟動')
        
        # 宣告參數
        self.declare_parameters()
        
        # TF 設置
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.static_broadcaster = StaticTransformBroadcaster(self)
        
        # 資料儲存
        self.poses_2d = []
        self.timestamps_2d = []
        self.poses_3d = []
        self.timestamps_3d = []
        self.T_base_camera = None
        self.T_alignment = None
        
        # 載入資料
        self.load_static_data()
        
        # 開始收集 2D 位姿
        self.collection_timer = self.create_timer(0.1, self.collect_2d_pose_callback)
        self.logger.info('開始收集 2D 位姿...')
    
    def declare_parameters(self):
        """宣告 ROS 參數"""
        self.declare_parameter('uni3r_poses_file', 'poses.txt')
        self.declare_parameter('base_camera_calib_file', 'camera_calibration.yaml')
        self.declare_parameter('output_file', 'map_alignment.yaml')
        
        self.declare_parameter('map_2d_frame', 'map_2d')
        self.declare_parameter('map_3d_frame', 'map_3d')
        self.declare_parameter('base_frame', 'base_footprint')
        
        self.declare_parameter('collection_time', 30.0)  # 收集時間 (秒)
        self.declare_parameter('min_poses', 50)  # 最少位姿數
        self.declare_parameter('max_time_diff', 0.05)  # 時間同步閾值
        self.declare_parameter('ransac_iterations', 1000)
        self.declare_parameter('inlier_threshold', 0.10)  # 10 cm
    
    def load_static_data(self):
        """載入靜態資料 (3D 位姿、外參)"""
        # 載入 Uni3R 位姿
        uni3r_file = self.get_parameter('uni3r_poses_file').value
        self.logger.info(f'載入 Uni3R 位姿: {uni3r_file}')
        
        try:
            self.timestamps_3d, self.poses_3d = DataLoader.load_uni3r_poses(uni3r_file)
            self.logger.info(f'成功載入 {len(self.poses_3d)} 個 3D 位姿')
        except Exception as e:
            self.logger.error(f'載入 Uni3R 位姿失敗: {e}')
            raise
        
        # 載入外參
        calib_file = self.get_parameter('base_camera_calib_file').value
        self.logger.info(f'載入相機外參: {calib_file}')
        
        try:
            self.T_base_camera = DataLoader.load_calibration(calib_file)
            self.logger.info('成功載入相機外參')
            self.logger.info(f'平移: {self.T_base_camera[:3, 3]}')
        except Exception as e:
            self.logger.error(f'載入外參失敗: {e}')
            raise
    
    def collect_2d_pose_callback(self):
        """收集 2D SLAM 位姿的回調函數"""
        try:
            # 查詢 TF
            map_frame = self.get_parameter('map_2d_frame').value
            base_frame = self.get_parameter('base_frame').value
            
            trans = self.tf_buffer.lookup_transform(
                map_frame,
                base_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.1)
            )
            
            # 轉換為矩陣並儲存
            T = transform_to_matrix(trans)
            self.poses_2d.append(T)
            
            # 儲存時間戳 (轉換為秒)
            stamp = trans.header.stamp
            timestamp = stamp.sec + stamp.nanosec * 1e-9
            self.timestamps_2d.append(timestamp)
            
            # 檢查是否收集足夠
            collection_time = self.get_parameter('collection_time').value
            min_poses = self.get_parameter('min_poses').value
            
            if len(self.poses_2d) >= min_poses:
                elapsed = self.timestamps_2d[-1] - self.timestamps_2d[0]
                
                if elapsed >= collection_time:
                    self.logger.info(f'已收集 {len(self.poses_2d)} 個位姿，開始對齊...')
                    self.collection_timer.cancel()
                    self.perform_alignment()
                else:
                    remaining = collection_time - elapsed
                    if len(self.poses_2d) % 50 == 0:  # 每 50 個位姿報告一次
                        self.logger.info(f'已收集 {len(self.poses_2d)} 個位姿，剩餘時間 {remaining:.1f}s')
        
        except Exception as e:
            # TF 查詢失敗是正常的，不需要每次都報錯
            pass
    
    def perform_alignment(self):
        """執行地圖對齊"""
        self.logger.info('=' * 50)
        self.logger.info('開始地圖對齊流程')
        self.logger.info('=' * 50)
        
        # 1. 時間同步
        self.logger.info('步驟 1: 時間同步')
        max_time_diff = self.get_parameter('max_time_diff').value
        
        synced_pairs = TrajectorySync.synchronize(
            np.array(self.timestamps_2d),
            self.poses_2d,
            self.timestamps_3d,
            self.poses_3d,
            max_time_diff=max_time_diff
        )
        
        sync_report = TrajectorySync.report_sync_quality(synced_pairs)
        self.logger.info(sync_report)
        
        if len(synced_pairs) < 10:
            self.logger.error('同步位姿對數過少，無法進行對齊')
            return
        
        # 2. 座標轉換
        self.logger.info('步驟 2: 將相機位姿轉為底盤位姿')
        poses_3d_camera = [p['pose_3d'] for p in synced_pairs]
        poses_3d_base = CoordinateTransform.transform_trajectory(
            poses_3d_camera,
            self.T_base_camera
        )
        poses_2d_synced = [p['pose_2d'] for p in synced_pairs]
        
        # 3. 軌跡品質檢查
        self.logger.info('步驟 3: 軌跡品質檢查')
        quality_report = TrajectoryQualityChecker.generate_report(
            poses_2d_synced,
            poses_3d_base
        )
        self.logger.info(quality_report)
        
        # 4. RANSAC 對齊
        self.logger.info('步驟 4: RANSAC robust 對齊')
        ransac_iter = self.get_parameter('ransac_iterations').value
        inlier_thresh = self.get_parameter('inlier_threshold').value
        
        T_alignment, inliers = MapAlignment.robust_alignment(
            poses_2d_synced,
            poses_3d_base,
            ransac_iterations=ransac_iter,
            inlier_threshold=inlier_thresh
        )
        
        inlier_ratio = inliers.sum() / len(inliers)
        self.logger.info(f'內點比例: {inlier_ratio*100:.1f}% ({inliers.sum()}/{len(inliers)})')
        
        if inlier_ratio < 0.5:
            self.logger.warning('內點比例過低，對齊結果可能不可靠')
        
        # 5. 精化對齊
        self.logger.info('步驟 5: 非線性精化')
        inlier_poses_2d = [poses_2d_synced[i] for i in np.where(inliers)[0]]
        inlier_poses_3d = [poses_3d_base[i] for i in np.where(inliers)[0]]
        
        T_refined = MapAlignment.refine_3d_alignment(
            inlier_poses_2d,
            inlier_poses_3d,
            T_alignment,
            loss_type='huber'
        )
        
        self.T_alignment = T_refined
        
        # 6. 誤差報告
        self.logger.info('步驟 6: 計算對齊誤差')
        error_dict = AlignmentError.compute_error(
            T_refined,
            inlier_poses_2d,
            inlier_poses_3d
        )
        error_report = AlignmentError.generate_report(error_dict)
        self.logger.info(error_report)
        
        # 7. 發布 TF
        self.logger.info('步驟 7: 發布靜態 TF')
        self.publish_alignment_tf()
        
        # 8. 儲存結果
        self.logger.info('步驟 8: 儲存對齊結果')
        self.save_alignment_result(error_dict)
        
        self.logger.info('=' * 50)
        self.logger.info('地圖對齊完成！')
        self.logger.info('=' * 50)
    
    def publish_alignment_tf(self):
        """發布對齊的靜態 TF"""
        if self.T_alignment is None:
            self.logger.error('尚未完成對齊，無法發布 TF')
            return
        
        map_2d_frame = self.get_parameter('map_2d_frame').value
        map_3d_frame = self.get_parameter('map_3d_frame').value
        
        trans = matrix_to_transform(
            self.T_alignment,
            map_2d_frame,
            map_3d_frame,
            self.get_clock().now().to_msg()
        )
        
        self.static_broadcaster.sendTransform(trans)
        self.logger.info(f'已發布靜態 TF: {map_2d_frame} -> {map_3d_frame}')
    
    def save_alignment_result(self, error_dict: Dict):
        """儲存對齊結果"""
        output_file = self.get_parameter('output_file').value
        
        # 準備儲存資料
        result = {
            'timestamp': self.get_clock().now().to_msg().sec,
            'map_2d_frame': self.get_parameter('map_2d_frame').value,
            'map_3d_frame': self.get_parameter('map_3d_frame').value,
            'transformation': {
                'matrix': self.T_alignment.tolist(),
                'translation': self.T_alignment[:3, 3].tolist(),
                'rotation_quaternion': rotation_matrix_to_quaternion(
                    self.T_alignment[:3, :3]
                ).tolist()
            },
            'error_statistics': error_dict,
            'parameters': {
                'max_time_diff': self.get_parameter('max_time_diff').value,
                'ransac_iterations': self.get_parameter('ransac_iterations').value,
                'inlier_threshold': self.get_parameter('inlier_threshold').value
            }
        }
        
        # 儲存為 YAML
        with open(output_file, 'w') as f:
            yaml.dump(result, f, default_flow_style=False)
        
        self.logger.info(f'對齊結果已儲存至: {output_file}')
        
        # 同時儲存為 static_transform_publisher 指令
        cmd_file = output_file.replace('.yaml', '_command.sh')
        T = self.T_alignment
        q = rotation_matrix_to_quaternion(T[:3, :3])
        
        cmd = f"""#!/bin/bash
# 自動生成的靜態 TF 發布指令
# 生成時間: {self.get_clock().now().to_msg().sec}

ros2 run tf2_ros static_transform_publisher \\
  {T[0,3]:.6f} {T[1,3]:.6f} {T[2,3]:.6f} \\
  {q[0]:.6f} {q[1]:.6f} {q[2]:.6f} {q[3]:.6f} \\
  {self.get_parameter('map_2d_frame').value} \\
  {self.get_parameter('map_3d_frame').value}
"""
        
        with open(cmd_file, 'w') as f:
            f.write(cmd)
        
        import os
        os.chmod(cmd_file, 0o755)
        
        self.logger.info(f'TF 發布指令已儲存至: {cmd_file}')


# ============================================================================
# 獨立工具：離線對齊
# ============================================================================

class OfflineAlignment:
    """
    離線對齊工具
    
    用於在沒有 ROS 環境下進行對齊計算
    """
    
    @staticmethod
    def align_from_files(poses_2d_file: str,
                        poses_3d_file: str,
                        calibration_file: str,
                        output_file: str = 'alignment_result.yaml'):
        """
        從檔案執行對齊
        
        Args:
            poses_2d_file: 2D 位姿檔案 (格式: timestamp tx ty tz qx qy qz qw)
            poses_3d_file: 3D 位姿檔案 (Uni3R 格式)
            calibration_file: 外參檔案
            output_file: 輸出檔案
        """
        print('=' * 50)
        print('離線地圖對齊工具')
        print('=' * 50)
        
        # 載入資料
        print(f'\n載入 2D 位姿: {poses_2d_file}')
        timestamps_2d, poses_2d = DataLoader.load_uni3r_poses(poses_2d_file)
        print(f'✓ 已載入 {len(poses_2d)} 個 2D 位姿')
        
        print(f'\n載入 3D 位姿: {poses_3d_file}')
        timestamps_3d, poses_3d = DataLoader.load_uni3r_poses(poses_3d_file)
        print(f'✓ 已載入 {len(poses_3d)} 個 3D 位姿')
        
        print(f'\n載入外參: {calibration_file}')
        T_base_camera = DataLoader.load_calibration(calibration_file)
        print(f'✓ 已載入外參')
        
        # 時間同步
        print('\n時間同步...')
        synced = TrajectorySync.synchronize(
            timestamps_2d, poses_2d,
            timestamps_3d, poses_3d
        )
        print(TrajectorySync.report_sync_quality(synced))
        
        # 座標轉換
        print('\n座標轉換...')
        poses_3d_camera = [p['pose_3d'] for p in synced]
        poses_3d_base = CoordinateTransform.transform_trajectory(
            poses_3d_camera, T_base_camera
        )
        poses_2d_synced = [p['pose_2d'] for p in synced]
        
        # 品質檢查
        print('\n軌跡品質檢查...')
        print(TrajectoryQualityChecker.generate_report(poses_2d_synced, poses_3d_base))
        
        # 對齊
        print('\nRANSAC 對齊...')
        T_alignment, inliers = MapAlignment.robust_alignment(
            poses_2d_synced, poses_3d_base
        )
        print(f'內點: {inliers.sum()}/{len(inliers)} ({inliers.sum()/len(inliers)*100:.1f}%)')
        
        # 精化
        print('\n非線性精化...')
        inlier_poses_2d = [poses_2d_synced[i] for i in np.where(inliers)[0]]
        inlier_poses_3d = [poses_3d_base[i] for i in np.where(inliers)[0]]
        
        T_refined = MapAlignment.refine_3d_alignment(
            inlier_poses_2d, inlier_poses_3d, T_alignment
        )
        
        # 誤差報告
        print('\n對齊誤差...')
        error_dict = AlignmentError.compute_error(
            T_refined, inlier_poses_2d, inlier_poses_3d
        )
        print(AlignmentError.generate_report(error_dict))
        
        # 儲存結果
        result = {
            'transformation': {
                'matrix': T_refined.tolist(),
                'translation': T_refined[:3, 3].tolist(),
                'rotation_quaternion': rotation_matrix_to_quaternion(
                    T_refined[:3, :3]
                ).tolist()
            },
            'error_statistics': error_dict
        }
        
        with open(output_file, 'w') as f:
            yaml.dump(result, f)
        
        print(f'\n✓ 結果已儲存至: {output_file}')
        print('=' * 50)
        
        return T_refined, error_dict


# ============================================================================
# 主程式
# ============================================================================

def main(args=None):
    """主程式入口"""
    rclpy.init(args=args)
    
    try:
        node = MapAlignmentNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'錯誤: {e}')
        import traceback
        traceback.print_exc()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()