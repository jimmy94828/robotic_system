#!/usr/bin/env python3
"""
NPZ 檔案讀取工具
功能：讀取並顯示 .npz 檔案的內容（欄位、資料內容）
使用方式：
    python3 read_npz.py <npz_file_path>
    或直接執行會使用預設檔案
"""

import numpy as np
import sys
import os


def read_npz(file_path):
    """讀取並顯示 NPZ 檔案內容"""
    print("=" * 70)
    print(f" 讀取檔案: {file_path}")
    print("=" * 70)
    
    try:
        # 讀取 NPZ 檔案
        data = np.load(file_path)
        
        # 取得所有的 key（欄位名稱）
        keys = data.files
        
        print(f"\n NPZ 檔案資訊:")
        print(f"   - 陣列數量: {len(keys)}")
        print(f"   - 陣列清單: {', '.join(keys)}")
        
        # 顯示每個陣列的詳細資訊
        print(f"\n  陣列詳細資訊:")
        print("-" * 70)
        
        for key in keys:
            array = data[key]
            print(f"\n 欄位: '{key}'")
            print(f"   - 形狀 (Shape):    {array.shape}")
            print(f"   - 資料型別 (Dtype): {array.dtype}")
            print(f"   - 總元素數:         {array.size}")
            print(f"   - 記憶體大小:       {array.nbytes / 1024:.2f} KB")
            
            # 顯示資料預覽
            print(f"\n    資料預覽:")
            
            # 根據陣列維度調整顯示方式
            if array.ndim == 1:
                # 1D 陣列
                preview_count = min(10, len(array))
                print(f"      前 {preview_count} 個元素:")
                for i in range(preview_count):
                    print(f"      [{i}] {array[i]}")
                if len(array) > preview_count:
                    print(f"      ... (共 {len(array)} 個元素)")
            
            elif array.ndim == 2:
                # 2D 陣列
                preview_rows = min(5, array.shape[0])
                print(f"      前 {preview_rows} 行:")
                for i in range(preview_rows):
                    row_str = str(array[i])
                    if len(row_str) > 80:
                        row_str = row_str[:77] + "..."
                    print(f"      [{i}] {row_str}")
                if array.shape[0] > preview_rows:
                    print(f"      ... (共 {array.shape[0]} 行)")
            
            elif array.ndim == 3:
                # 3D 陣列
                print(f"      形狀: {array.shape}")
                print(f"      第一個切片 ([:2, :2, :]):")
                preview = array[:2, :2, :]
                print(f"      {preview}")
            
            else:
                # 高維陣列
                print(f"      形狀: {array.shape}")
                flat = array.flatten()
                print(f"      展平後前 10 個元素: {flat[:10]}")
            
            # 顯示統計資訊（僅針對數值型別）
            if np.issubdtype(array.dtype, np.number):
                print(f"\n    統計資訊:")
                try:
                    print(f"      最小值 (min):   {array.min()}")
                    print(f"      最大值 (max):   {array.max()}")
                    print(f"      平均值 (mean):  {array.mean():.6f}")
                    print(f"      標準差 (std):   {array.std():.6f}")
                    
                    # 對於整數陣列，顯示唯一值
                    if np.issubdtype(array.dtype, np.integer):
                        unique_vals = np.unique(array)
                        if len(unique_vals) <= 20:
                            print(f"      唯一值:         {unique_vals}")
                        else:
                            print(f"      唯一值數量:     {len(unique_vals)}")
                except:
                    pass
        
        print("\n" + "=" * 70)
        print(" 讀取完成")
        
        # 關閉檔案
        data.close()
        
        return data
        
    except FileNotFoundError:
        print(f" 錯誤: 檔案不存在: {file_path}")
        sys.exit(1)
    except Exception as e:
        print(f" 讀取失敗: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


def compare_common_formats(file_path):
    """比較常見的 NPZ 檔案格式"""
    data = np.load(file_path)
    keys = set(data.files)
    
    print("\n" + "=" * 70)
    print("🔍 格式識別:")
    
    # SLAM/Gaussian Splatting 格式
    slam_keys = {'means3D', 'unnorm_rotations', 'log_scales', 'logit_opacities', 
                 'rgb_colors', 'semantic_ids', 'semantic_colors'}
    if slam_keys.issubset(keys):
        print("    檢測到: SLAM/Gaussian Splatting 地圖格式")
        print("      包含: 3D 位置, 旋轉, 縮放, 不透明度, RGB, 語義資訊")
    
    # Panoptic SLAM 格式
    panoptic_keys = {'pts', 'pan', 'rgb'}
    if panoptic_keys.issubset(keys):
        print("    檢測到: Panoptic SLAM 格式")
        print("      包含: 點座標, Panoptic ID, RGB 顏色")
    
    # 簡單點雲格式
    if 'points' in keys and len(keys) <= 3:
        print("    檢測到: 簡單點雲格式")
        print("      包含: 點座標" + (" 和顏色" if 'colors' in keys else ""))
    
    data.close()


def main():
    # 取得檔案路徑
    if len(sys.argv) > 1:
        file_path = sys.argv[1]
    else:
        # 預設檔案路徑
        file_path = '/home/weichen/project/robot_ws/data/params.npz'
        print(f"  未指定檔案，使用預設路徑: {file_path}")
    
    # 檢查檔案是否存在
    if not os.path.exists(file_path):
        print(f" 錯誤: 檔案不存在: {file_path}")
        print(f"\n使用方式: python3 {sys.argv[0]} <npz_file_path>")
        
        # 列出一些可能的檔案
        print(f"\n 提示: robot_ws 中可能的 NPZ 檔案:")
        possible_paths = [
            '/home/weichen/project/robot_ws/data/params.npz',
            '/home/weichen/project/robot_ws/data/lab/*.npz',
            '/home/weichen/project/Util/*.npz'
        ]
        for path in possible_paths:
            print(f"   - {path}")
        
        sys.exit(1)
    
    # 讀取並顯示檔案內容
    read_npz(file_path)
    
    # 嘗試識別檔案格式
    try:
        compare_common_formats(file_path)
    except:
        pass


if __name__ == "__main__":
    main()
