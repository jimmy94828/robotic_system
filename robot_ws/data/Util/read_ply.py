#!/usr/bin/env python3
"""
PLY 檔案讀取工具
功能：讀取並顯示 .ply 檔案的內容（欄位、資料內容）
使用方式：
    python3 read_ply.py <ply_file_path>
    或直接執行會使用預設檔案
"""

import numpy as np
import sys
import os

try:
    from plyfile import PlyData, PlyElement
    HAVE_PLYFILE = True
except ImportError:
    HAVE_PLYFILE = False

try:
    import open3d as o3d
    HAVE_OPEN3D = True
except ImportError:
    HAVE_OPEN3D = False


def read_ply_with_plyfile(file_path):
    """使用 plyfile 庫讀取 PLY 檔案（可讀取所有自訂欄位）"""
    print("=" * 70)
    print(f"📂 讀取檔案: {file_path}")
    print("=" * 70)
    
    # 讀取 PLY 檔案
    ply_data = PlyData.read(file_path)
    
    # 顯示檔案資訊
    print(f"\n📊 PLY 檔案結構:")
    print(f"   - 元素數量: {len(ply_data.elements)}")
    
    for element in ply_data.elements:
        print(f"\n🔹 元素名稱: '{element.name}'")
        print(f"   - 資料筆數: {element.count}")
        print(f"   - 欄位數量: {len(element.properties)}")
        print(f"   - 欄位清單:")
        
        for prop in element.properties:
            print(f"      • {prop.name:20s} (型別: {prop.val_dtype})")
        
        # 顯示資料預覽
        if element.count > 0:
            print(f"\n   📋 資料預覽 (前 5 筆):")
            data_array = element.data
            
            # 取得前 5 筆資料
            preview_count = min(5, element.count)
            for i in range(preview_count):
                print(f"      [{i}] {data_array[i]}")
            
            if element.count > 5:
                print(f"      ... (共 {element.count} 筆資料)")
            
            # 顯示統計資訊（針對數值欄位）
            print(f"\n   📈 數值統計:")
            for prop in element.properties:
                if prop.val_dtype in [np.float32, np.float64, np.int32, np.int64]:
                    values = data_array[prop.name]
                    print(f"      {prop.name:20s}: min={values.min():.4f}, max={values.max():.4f}, mean={values.mean():.4f}")
    
    print("\n" + "=" * 70)
    return ply_data


def read_ply_with_open3d(file_path):
    """使用 Open3D 庫讀取 PLY 檔案（僅支援標準欄位）"""
    print("=" * 70)
    print(f"📂 讀取檔案: {file_path}")
    print("=" * 70)
    
    # 讀取點雲
    pcd = o3d.io.read_point_cloud(file_path)
    
    if not pcd.has_points():
        print("❌ 錯誤: 點雲檔案沒有任何點資料")
        return None
    
    # 顯示基本資訊
    print(f"\n📊 點雲資訊:")
    print(f"   - 點數量: {len(pcd.points)}")
    
    # 顯示可用欄位
    print(f"\n🔹 可用欄位:")
    has_data = []
    
    if pcd.has_points():
        has_data.append("points (xyz)")
        points = np.asarray(pcd.points)
        print(f"   • points: shape={points.shape}, dtype={points.dtype}")
        print(f"      預覽: {points[:3]}")
    
    if pcd.has_colors():
        has_data.append("colors")
        colors = np.asarray(pcd.colors)
        print(f"   • colors: shape={colors.shape}, dtype={colors.dtype}")
        print(f"      預覽: {colors[:3]}")
    
    if pcd.has_normals():
        has_data.append("normals")
        normals = np.asarray(pcd.normals)
        print(f"   • normals: shape={normals.shape}, dtype={normals.dtype}")
        print(f"      預覽: {normals[:3]}")
    
    print(f"\n   ⚠️  注意: Open3D 只能讀取標準欄位 (xyz, rgb, normals)")
    print(f"          如需讀取自訂欄位，請使用 plyfile 庫")
    
    # 顯示統計資訊
    if pcd.has_points():
        print(f"\n📈 座標統計:")
        points = np.asarray(pcd.points)
        print(f"   X: min={points[:, 0].min():.4f}, max={points[:, 0].max():.4f}")
        print(f"   Y: min={points[:, 1].min():.4f}, max={points[:, 1].max():.4f}")
        print(f"   Z: min={points[:, 2].min():.4f}, max={points[:, 2].max():.4f}")
    
    print("\n" + "=" * 70)
    return pcd


def main():
    # 取得檔案路徑
    if len(sys.argv) > 1:
        file_path = sys.argv[1]
    else:
        # 預設檔案路徑
        file_path = '/home/weichen/project/Util/Final_SEM_GS.ply'
        print(f"⚠️  未指定檔案，使用預設路徑: {file_path}")
    
    # 檢查檔案是否存在
    if not os.path.exists(file_path):
        print(f"❌ 錯誤: 檔案不存在: {file_path}")
        print(f"\n使用方式: python3 {sys.argv[0]} <ply_file_path>")
        sys.exit(1)
    
    # 選擇讀取方式
    print(f"\n🔧 可用的讀取方式:")
    if HAVE_PLYFILE:
        print(f"   ✅ plyfile (推薦 - 可讀取所有自訂欄位)")
    else:
        print(f"   ❌ plyfile (未安裝: pip install plyfile)")
    
    if HAVE_OPEN3D:
        print(f"   ✅ open3d (僅支援標準欄位)")
    else:
        print(f"   ❌ open3d (未安裝: pip install open3d)")
    
    print()
    
    # 優先使用 plyfile，因為它能讀取自訂欄位
    if HAVE_PLYFILE:
        try:
            read_ply_with_plyfile(file_path)
        except Exception as e:
            print(f"❌ plyfile 讀取失敗: {e}")
            if HAVE_OPEN3D:
                print(f"\n🔄 嘗試使用 Open3D...")
                read_ply_with_open3d(file_path)
    elif HAVE_OPEN3D:
        read_ply_with_open3d(file_path)
    else:
        print("❌ 錯誤: 請安裝 plyfile 或 open3d")
        print("   pip install plyfile")
        print("   或")
        print("   pip install open3d")
        sys.exit(1)


if __name__ == "__main__":
    main()
