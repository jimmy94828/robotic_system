#!/usr/bin/env python3
"""
從 Kachaka 導出地圖到本地文件系統，以便進行後續的語義地圖對齊
使用方法:
    python3 export_kachaka_map.py <ROBOT_IP>
例如:
    python3 export_kachaka_map.py
"""
import kachaka_api
import yaml
import os
import sys

def export_kachaka_map(robot_ip, output_dir='./'):
    """從 Kachaka 機器人下載地圖並保存為 PNG 和 YAML 文件"""
    try:
        print(f"連接到 Kachaka: {robot_ip}")
        client = kachaka_api.KachakaApiClient(robot_ip)
        
        # 確保輸出目錄存在
        os.makedirs(output_dir, exist_ok=True)
        
        image_path = os.path.join(output_dir, 'kachaka_native.png')
        yaml_path = os.path.join(output_dir, 'kachaka_native.yaml')
        
        print("正在下載地圖...")
        map_data = client.get_png_map()
        
        # 保存圖像
        with open(image_path, "wb") as f:
            f.write(map_data.data)
        print(f"✅ 地圖圖像已保存: {image_path}")
        
        # 保存元數據
        map_metadata = {
            "image": "kachaka_native.png",  # 使用相對路徑
            "resolution": map_data.resolution,
            "origin": [map_data.origin.x, map_data.origin.y, 0.0],
            "negate": 0,
            "occupied_thresh": 0.65,
            "free_thresh": 0.196
        }
        
        with open(yaml_path, "w") as f:
            yaml.dump(map_metadata, f)
        print(f"✅ 地圖元數據已保存: {yaml_path}")
        
        print(f"\n地圖信息:")
        print(f"  分辨率: {map_data.resolution:.4f} m/pixel")
        print(f"  原點: ({map_data.origin.x:.2f}, {map_data.origin.y:.2f})")
        
        return True
        
    except Exception as e:
        print(f"❌ 導出失敗: {e}")
        import traceback
        traceback.print_exc()
        return False

def main():
    if len(sys.argv) < 2:
        print("用法: python3 export_kachaka_map.py <ROBOT_IP>")
        print("示例: python3 export_kachaka_map.py 192.168.0.157:26400")
        sys.exit(1)
    
    robot_ip = sys.argv[1]
    output_dir = './'  # 當前目錄
    
    success = export_kachaka_map(robot_ip, output_dir)
    
    if success:
        print("\n✅ 地圖導出成功！")
        print("現在可以運行: python3 auto_align.py")
    else:
        print("\n❌ 地圖導出失敗")
        sys.exit(1)

if __name__ == "__main__":
    main()
