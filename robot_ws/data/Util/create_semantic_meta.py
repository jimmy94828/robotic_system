#!/usr/bin/env python3
"""
將 semantic_legend.json 轉換為 object_query_server.py 需要的格式
"""

import json
import os

# 讀取原始的 semantic_legend.json
legend_path = '/home/acm118/robot_ws/data/Util/semantic_legend.json'

with open(legend_path, 'r') as f:
    legend = json.load(f)

# 提取所有類別名稱並生成 ID 映射
segments_info = []
for idx, (class_name, hsv_color) in enumerate(legend.items()):
    segments_info.append({
        "id": idx,
        "category_name": class_name,
        "class": class_name,
        "color_hsv": hsv_color
    })

# 創建新的 JSON 結構
output_json = {
    "segments_info": segments_info
}

# 保存
output_path = '/home/acm118/robot_ws/data/Util/semantic_meta.json'
with open(output_path, 'w') as f:
    json.dump(output_json, f, indent=2)

print(f"✅ 創建語義元數據文件: {output_path}")
print(f"   共 {len(segments_info)} 個類別:")
for seg in segments_info:
    print(f"   ID {seg['id']:2d}: {seg['category_name']}")
