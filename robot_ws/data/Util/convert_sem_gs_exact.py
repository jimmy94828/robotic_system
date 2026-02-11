#!/usr/bin/env python3
"""
使用精確 RGB 映射轉換 Final_SEM_GS.ply
這個版本直接使用實際顏色到類別的精確映射
"""

import numpy as np
import argparse
import os
import sys
from plyfile import PlyData
import json


def convert_with_exact_mapping(ply_path, output_path, mapping_path):
    """使用精確顏色映射轉換"""
    
    print("=" * 70)
    print(f"📂 讀取語義 PLY 文件: {ply_path}")
    print("=" * 70)
    
    # 1. 讀取 PLY
    ply_data = PlyData.read(ply_path)
    vertex_data = ply_data['vertex'].data
    num_points = len(vertex_data)
    print(f"✅ 成功讀取 {num_points:,} 個點")
    
    # 2. 提取 XYZ
    points = np.zeros((num_points, 3), dtype=np.float32)
    points[:, 0] = vertex_data['x']
    points[:, 1] = vertex_data['y']
    points[:, 2] = vertex_data['z']
    print(f"✅ 提取 XYZ 座標: {points.shape}")
    
    # 3. 提取 RGB
    sh_dc = np.zeros((num_points, 3), dtype=np.float32)
    sh_dc[:, 0] = vertex_data['f_dc_0']
    sh_dc[:, 1] = vertex_data['f_dc_1']
    sh_dc[:, 2] = vertex_data['f_dc_2']
    
    C0 = 0.28209479177387814
    rgb = sh_dc * C0 + 0.5
    rgb = np.clip(rgb, 0, 1)
    print(f"✅ 從球諧係數轉換 RGB: {rgb.shape}")
    
    # 4. 讀取精確映射
    print(f"\n📂 讀取精確顏色映射: {mapping_path}")
    with open(mapping_path, 'r') as f:
        mapping_data = json.load(f)
    
    color_mappings = mapping_data['color_mapping']
    print(f"✅ 載入 {len(color_mappings)} 個顏色映射")
    
    # 5. 建立 RGB -> 類別 ID 的映射
    rgb_to_id = {}
    id_to_name = {}
    
    for i, mapping in enumerate(color_mappings):
        rgb_key = tuple(np.round(mapping['rgb'], 3))
        category = mapping['category']
        rgb_to_id[rgb_key] = i
        id_to_name[i] = category
    
    print(f"\n🎨 開始精確顏色匹配（RGB 空間，容差 0.001）...")
    
    # 6. 匹配每個點（使用容差）
    semantic_ids = np.zeros(num_points, dtype=np.int32)
    
    # 準備所有映射顏色的數組
    mapping_rgbs = np.array([m['rgb'] for m in color_mappings])  # (16, 3)
    
    matched_count = 0
    batch_size = 100000
    
    for batch_start in range(0, num_points, batch_size):
        batch_end = min(batch_start + batch_size, num_points)
        batch_rgb = rgb[batch_start:batch_end]  # (batch_size, 3)
        
        # 計算批次中每個點到所有映射顏色的距離
        # batch_rgb: (batch_size, 3), mapping_rgbs: (16, 3)
        # distances: (batch_size, 16)
        distances = np.sqrt(np.sum((batch_rgb[:, np.newaxis, :] - mapping_rgbs[np.newaxis, :, :]) ** 2, axis=2))
        
        # 找到最近的顏色
        closest_indices = np.argmin(distances, axis=1)  # (batch_size,)
        min_distances = np.min(distances, axis=1)  # (batch_size,)
        
        # 分配 ID
        semantic_ids[batch_start:batch_end] = closest_indices
        
        # 統計匹配（距離小於 0.001 視為精確匹配）
        matched_count += np.sum(min_distances < 0.001)
        
        if batch_end % 500000 == 0 or batch_end == num_points:
            print(f"   處理進度: {batch_end}/{num_points} ({100*batch_end/num_points:.1f}%)")
    
    print(f"   ✅ 完成匹配")
    print(f"   精確匹配率: {100*matched_count/num_points:.2f}% ({matched_count:,}/{num_points:,})")
    print(f"   近似匹配: 所有點都被分配到最接近的類別")
    
    # 7. 統計類別分佈
    unique_ids = np.unique(semantic_ids)
    print(f"\n✅ 語義分類完成:")
    print(f"   - 語義 ID 範圍: [{semantic_ids.min()}, {semantic_ids.max()}]")
    print(f"   - 檢測到的類別數: {len(unique_ids)}")
    
    id_counts = np.bincount(semantic_ids)
    print(f"   - 類別分佈:")
    for i, count in enumerate(id_counts):
        if count > 0:
            pct = 100 * count / num_points
            class_name = id_to_name.get(i, 'unknown')
            print(f"      ID {i:2d} ({class_name:15s}): {count:7,} 點 ({pct:5.2f}%)")
    
    # 8. 保存 NPZ
    save_dict = {
        'means3D': points,
        'pts': points,
        'pan': semantic_ids,
        'semantic_ids': semantic_ids,
        'rgb': rgb
    }
    
    np.savez_compressed(output_path, **save_dict)
    print(f"\n✅ 成功保存 NPZ 文件: {output_path}")
    
    # 9. 創建元數據 JSON
    json_output = output_path.replace('.npz', '_meta.json')
    segments_info = []
    for idx, name in id_to_name.items():
        if idx < len(id_counts) and id_counts[idx] > 0:
            segments_info.append({
                "id": int(idx),
                "category_name": name,
                "class": name,
                "point_count": int(id_counts[idx])
            })
    
    output_json = {"segments_info": segments_info}
    with open(json_output, 'w') as f:
        json.dump(output_json, f, indent=2)
    
    print(f"✅ 保存語義元數據: {json_output}")
    
    # 10. 驗證
    print("\n" + "=" * 70)
    print("📋 驗證保存的 NPZ 文件:")
    print("=" * 70)
    
    loaded = np.load(output_path)
    for key in loaded.keys():
        data = loaded[key]
        print(f"   • {key:20s}: shape={data.shape}, dtype={data.dtype}")
    
    print("\n" + "=" * 70)
    print("✅ 轉換完成！")
    print("=" * 70)
    
    return True


def main():
    parser = argparse.ArgumentParser(description='使用精確 RGB 映射轉換 Final_SEM_GS.ply')
    parser.add_argument('--input_ply', default='/home/acm118/robot_ws/data/Util/Final_GS.ply', help='輸入 PLY 文件')
    parser.add_argument('--output_npz', default='/home/acm118/robot_ws/data/Util/Final_GS_converted.npz', help='輸出 NPZ 文件')
    parser.add_argument('--mapping', '-m', default='actual_color_mapping.json', 
                       help='顏色映射 JSON 文件（默認: actual_color_mapping.json）')
    
    args = parser.parse_args()
    
    success = convert_with_exact_mapping(args.input_ply, args.output_npz, args.mapping)
    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()
