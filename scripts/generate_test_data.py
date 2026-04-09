#!/usr/bin/env python3
"""
generate_test_data.py
生成 LiDAR-Camera 外参标定的伪测试数据:
  1) 一张伪相机照片 (PNG)  — 基于鱼眼内参 + 已知外参
  2) 一个伪点云文件 (PCD) — 模拟 360°×90° 雷达扫描

标定板规格 (fast_calib):
  - 板面尺寸: 1400mm × 1000mm
  - 四角 ArUco: 200mm 边长, 距板边 50mm
  - 中心 2×2 圆孔: 直径 240mm, 水平间距 500mm, 垂直间距 400mm

坐标系约定:
  - 相机坐标系: Z 朝前, X 朝右, Y 朝下
  - 雷达坐标系: X 朝前, Y 朝左, Z 朝上
  - 雷达在相机正上方 (偏移约 0.15m)
"""

import numpy as np
import cv2
import yaml
import struct
import os
import sys

# ============================================================
#                     参数配置区
# ============================================================

# 输出路径
OUTPUT_DIR = os.path.join(os.path.dirname(__file__), "..", "data")
OUTPUT_IMAGE = os.path.join(OUTPUT_DIR, "test_image.png")
OUTPUT_PCD = os.path.join(OUTPUT_DIR, "test_cloud.pcd")

# --- 相机内参 (KANNALA_BRANDT 鱼眼模型) ---
IMAGE_W, IMAGE_H = 1920, 1536
MU, MV = 509.4090, 509.4703
U0, V0 = 942.1229, 781.1226
K2, K3, K4, K5 = 0.13513, -0.03706, 0.000775, 0.000286

# --- 已知外参: LiDAR -> Camera ---
# 雷达在相机正上方约 0.15m, 雷达 X 朝前 = 相机 Z 朝前
# 旋转: R_lidar_to_cam 将 (X前,Y左,Z上) -> (X右,Y下,Z前)
T_LIDAR_TO_CAM = np.array([
    [ 0.0,  1.0,  0.0,  0.0  ],   # cam_X = lidar_Y (但取反: 左->右)
    [ 0.0,  0.0, -1.0, -0.15 ],   # cam_Y = -lidar_Z (上->下), 平移 -0.15m
    [ 1.0,  0.0,  0.0,  0.0  ],   # cam_Z = lidar_X (前->前)
    [ 0.0,  0.0,  0.0,  1.0  ]
], dtype=np.float64)
# 修正: 雷达Y朝左, 相机X朝右, 所以 cam_X = -lidar_Y
T_LIDAR_TO_CAM[0, 1] = -1.0

# --- 标定板参数 (单位: 米) ---
BOARD_W = 1.4       # 板宽 (水平/X方向)
BOARD_H = 1.0       # 板高 (垂直/Y方向)
BOARD_DIST = 1.2    # 板面距相机距离 (Z方向), 参考实物约 1.2m

MARKER_SIZE = 0.2   # ArUco 边长
MARKER_MARGIN = 0.05  # ArUco 距板边距离

CIRCLE_DIAMETER = 0.24
CIRCLE_RADIUS = CIRCLE_DIAMETER / 2.0
CIRCLE_DX = 0.5     # 水平圆心距
CIRCLE_DY = 0.4     # 垂直圆心距

# --- 雷达参数 ---
LIDAR_H_RESOLUTION = 2048   # 水平方向点数
LIDAR_V_RESOLUTION = 390    # 垂直方向点数 (总约 800k)
LIDAR_H_FOV = 360.0         # 水平 FOV (度)
LIDAR_V_FOV = 90.0          # 垂直 FOV (度), -45° ~ +45°
CYLINDER_RADIUS = 4.0       # 背景圆柱体半径 (米)
LIDAR_INTENSITY = 100.0     # 反射率


# ============================================================
#              鱼眼投影 (KANNALA_BRANDT 模型)
# ============================================================

def kannala_brandt_project(pts_3d):
    """
    将相机坐标系下的 3D 点投影到图像平面 (KANNALA_BRANDT 模型).
    pts_3d: (N, 3) numpy array
    返回: (N, 2) 像素坐标, (N,) 有效性掩码
    """
    x, y, z = pts_3d[:, 0], pts_3d[:, 1], pts_3d[:, 2]

    # 只投影 z > 0 的点 (在相机前方)
    valid = z > 0.01
    
    r_sq = x**2 + y**2
    r = np.sqrt(r_sq)
    theta = np.arctan2(r, z)  # 入射角

    # KANNALA_BRANDT 畸变模型: d(theta) = theta + k2*theta^3 + k3*theta^5 + ...
    theta2 = theta * theta
    theta3 = theta2 * theta
    theta5 = theta3 * theta2
    theta7 = theta5 * theta2
    theta9 = theta7 * theta2
    
    d_theta = theta + K2 * theta3 + K3 * theta5 + K4 * theta7 + K5 * theta9

    # 归一化方向
    r_safe = np.where(r > 1e-8, r, 1e-8)
    mx = d_theta * x / r_safe
    my = d_theta * y / r_safe

    # 像素坐标
    u = MU * mx + U0
    v = MV * my + V0

    # 检查是否在图像范围内
    valid = valid & (u >= 0) & (u < IMAGE_W) & (v >= 0) & (v < IMAGE_H)

    return np.stack([u, v], axis=-1), valid


# ============================================================
#              标定板 3D 几何 (相机坐标系)
# ============================================================

def get_board_corners_cam():
    """标定板四角在相机坐标系中的坐标 (Z=BOARD_DIST 平面)."""
    hw, hh = BOARD_W / 2, BOARD_H / 2
    # 相机坐标系: X 右, Y 下, Z 前
    return np.array([
        [-hw, -hh, BOARD_DIST],  # 左上
        [ hw, -hh, BOARD_DIST],  # 右上
        [ hw,  hh, BOARD_DIST],  # 右下
        [-hw,  hh, BOARD_DIST],  # 左下
    ])


def get_marker_corners_cam():
    """
    4 个 ArUco 标记的角点 (每个标记 4 个角, 共 16 个点).
    返回 list of 4 arrays, 每个 (4, 3).
    """
    hw, hh = BOARD_W / 2, BOARD_H / 2
    m = MARKER_MARGIN
    s = MARKER_SIZE
    
    # 标记中心位置 (相对于板中心)
    centers = [
        (-hw + m + s/2, -hh + m + s/2),  # 左上
        ( hw - m - s/2, -hh + m + s/2),  # 右上
        ( hw - m - s/2,  hh - m - s/2),  # 右下
        (-hw + m + s/2,  hh - m - s/2),  # 左下
    ]
    
    markers = []
    for cx, cy in centers:
        corners = np.array([
            [cx - s/2, cy - s/2, BOARD_DIST],
            [cx + s/2, cy - s/2, BOARD_DIST],
            [cx + s/2, cy + s/2, BOARD_DIST],
            [cx - s/2, cy + s/2, BOARD_DIST],
        ])
        markers.append(corners)
    return markers


def get_circle_centers_cam():
    """4 个圆孔中心在相机坐标系中的位置."""
    dx2, dy2 = CIRCLE_DX / 2, CIRCLE_DY / 2
    return np.array([
        [-dx2, -dy2, BOARD_DIST],
        [ dx2, -dy2, BOARD_DIST],
        [-dx2,  dy2, BOARD_DIST],
        [ dx2,  dy2, BOARD_DIST],
    ])


# ============================================================
#              生成伪照片
# ============================================================

def generate_image():
    """生成一张模拟的相机标定板照片."""
    print("[图像] 开始生成伪照片...")
    img = np.full((IMAGE_H, IMAGE_W, 3), 60, dtype=np.uint8)  # 深灰背景

    # --- 1. 绘制标定板主体 (鱼眼畸变下的弧形边缘) ---
    # 沿板面四条边密集采样, 每个点都经过鱼眼投影, 边缘自然弯曲
    N_EDGE = 80  # 每条边采样点数
    hw, hh = BOARD_W / 2, BOARD_H / 2
    edge_pts_3d = []
    # 上边: 左上 -> 右上
    for t in np.linspace(-hw, hw, N_EDGE):
        edge_pts_3d.append([t, -hh, BOARD_DIST])
    # 右边: 右上 -> 右下
    for t in np.linspace(-hh, hh, N_EDGE):
        edge_pts_3d.append([hw, t, BOARD_DIST])
    # 下边: 右下 -> 左下
    for t in np.linspace(hw, -hw, N_EDGE):
        edge_pts_3d.append([t, hh, BOARD_DIST])
    # 左边: 左下 -> 左上
    for t in np.linspace(hh, -hh, N_EDGE):
        edge_pts_3d.append([-hw, t, BOARD_DIST])

    edge_pts_3d = np.array(edge_pts_3d)
    pts_2d, valid = kannala_brandt_project(edge_pts_3d)
    if np.sum(valid) > N_EDGE * 2:
        poly = pts_2d[valid].astype(np.int32).reshape((-1, 1, 2))
        cv2.fillPoly(img, [poly], (220, 220, 220))  # 浅灰白色板面
        cv2.polylines(img, [poly], True, (100, 100, 100), 2)  # 边框

    # --- 2. 绘制 ArUco 标记 (使用的字典必须与 QRDetector 匹配) ---
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
    marker_corners_list = get_marker_corners_cam()
    # 标记 ID 顺序: 左上(1), 右上(2), 右下(4), 左下(3) -> 匹配 qr_detector.cpp:79
    marker_ids = [1, 2, 4, 3]
    
    for idx, corners_3d in enumerate(marker_corners_list):
        pts_2d, valid = kannala_brandt_project(corners_3d)
        if not np.all(valid):
            continue
        
        poly = pts_2d.astype(np.int32).reshape((-1, 1, 2))
        
        # 生成 ArUco 标记图像
        # 生成 ArUco 标记图像 (兼容不同 OpenCV 版本)
        try:
            marker_img = cv2.aruco.generateImageMarker(aruco_dict, marker_ids[idx], 200)
        except AttributeError:
            marker_img = np.zeros((200, 200), dtype=np.uint8)
            cv2.aruco.drawMarker(aruco_dict, marker_ids[idx], 200, marker_img, 1)
        marker_img_color = cv2.cvtColor(marker_img, cv2.COLOR_GRAY2BGR)
        
        # 透视变换: 将标准 marker 贴到投影后的四边形
        src_pts = np.array([[0, 0], [199, 0], [199, 199], [0, 199]], dtype=np.float32)
        dst_pts = pts_2d.astype(np.float32)
        
        M = cv2.getPerspectiveTransform(src_pts, dst_pts)
        warped = cv2.warpPerspective(marker_img_color, M, (IMAGE_W, IMAGE_H))
        
        # 创建掩码并合成
        mask = cv2.warpPerspective(
            np.ones_like(marker_img_color) * 255, M, (IMAGE_W, IMAGE_H)
        )
        mask_bool = mask[:, :, 0] > 128
        img[mask_bool] = warped[mask_bool]

    # --- 3. 绘制圆孔 (黑色圆) ---
    circle_centers = get_circle_centers_cam()
    for center_3d in circle_centers:
        # 采样圆周上的点来确定投影后的椭圆
        angles = np.linspace(0, 2 * np.pi, 64, endpoint=False)
        circle_pts = np.zeros((64, 3))
        circle_pts[:, 0] = center_3d[0] + CIRCLE_RADIUS * np.cos(angles)
        circle_pts[:, 1] = center_3d[1] + CIRCLE_RADIUS * np.sin(angles)
        circle_pts[:, 2] = center_3d[2]
        
        pts_2d, valid = kannala_brandt_project(circle_pts)
        if np.sum(valid) < 32:
            continue
        
        poly = pts_2d[valid].astype(np.int32).reshape((-1, 1, 2))
        cv2.fillPoly(img, [poly], (40, 40, 40))  # 深色表示孔洞

    # --- 4. 保存 ---
    os.makedirs(os.path.dirname(OUTPUT_IMAGE), exist_ok=True)
    cv2.imwrite(OUTPUT_IMAGE, img)
    print(f"[图像] ✓ 已保存: {OUTPUT_IMAGE}  ({IMAGE_W}x{IMAGE_H})")
    return img


# ============================================================
#              生成伪点云
# ============================================================

def generate_point_cloud():
    """
    生成模拟的 LiDAR 点云 (PCD 格式).
    
    雷达坐标系: X 朝前, Y 朝左, Z 朝上
    标定板直接放在雷达坐标系中:
      - 板面中心: (BOARD_DIST, 0, z_offset)  (在雷达正前方)
      - 板面法向: (-1, 0, 0)  (面朝雷达)
      - 板面宽度方向: Y 轴 (左右)
      - 板面高度方向: Z 轴 (上下)
    """
    print("[点云] 开始生成伪点云...")

    # --- 标定板在雷达坐标系中的位置 ---
    # 雷达在相机正上方 0.15m, 所以板中心在雷达系中 Z 略低
    board_center = np.array([BOARD_DIST, 0.0, -0.15])
    
    # 板面法向量 (面朝雷达, 即 -X 方向)
    board_normal = np.array([1.0, 0.0, 0.0])
    
    # 板面局部坐标轴
    board_y_dir = np.array([0.0, 1.0, 0.0])  # 宽度方向 (Y, 左右)
    board_z_dir = np.array([0.0, 0.0, 1.0])  # 高度方向 (Z, 上下)
    
    # --- 圆孔中心在雷达坐标系中的位置 ---
    # 圆心相对于板中心的偏移: 宽度沿 Y, 高度沿 Z
    dx2, dy2 = CIRCLE_DX / 2, CIRCLE_DY / 2
    circle_centers = [
        board_center + np.array([0, +dx2, +dy2]),  # 左上
        board_center + np.array([0, -dx2, +dy2]),  # 右上
        board_center + np.array([0, +dx2, -dy2]),  # 左下
        board_center + np.array([0, -dx2, -dy2]),  # 右下
    ]

    print(f"[点云] 板面中心 (雷达系): {board_center}")
    print(f"[点云] 圆孔中心 (雷达系):")
    for i, cc in enumerate(circle_centers):
        print(f"         圆孔{i}: ({cc[0]:.3f}, {cc[1]:.3f}, {cc[2]:.3f})")

    # --- 生成雷达射线并光线追踪 ---
    h_angles = np.linspace(0, 2 * np.pi, LIDAR_H_RESOLUTION, endpoint=False)
    v_angles = np.linspace(-np.radians(LIDAR_V_FOV / 2), 
                           np.radians(LIDAR_V_FOV / 2), 
                           LIDAR_V_RESOLUTION)
    
    points = []
    board_hits = 0
    total_rays = LIDAR_H_RESOLUTION * LIDAR_V_RESOLUTION
    print(f"[点云] 总射线数: {total_rays} ({LIDAR_H_RESOLUTION}x{LIDAR_V_RESOLUTION})")

    for v_ang in v_angles:
        cos_v = np.cos(v_ang)
        sin_v = np.sin(v_ang)
        for h_ang in h_angles:
            # 雷达坐标系射线方向: X前, Y左, Z上
            dx = np.cos(h_ang) * cos_v
            dy = np.sin(h_ang) * cos_v
            dz = sin_v
            ray_dir = np.array([dx, dy, dz])

            # --- 尝试与标定板平面求交 ---
            # 平面方程: dot(P - board_center, board_normal) = 0
            denom = np.dot(ray_dir, board_normal)
            hit_board = False
            
            if abs(denom) > 1e-6:
                t = np.dot(board_center, board_normal) / denom
                if t > 0.1:  # 排除过近的交点
                    hit_pt = ray_dir * t
                    
                    # 在板面局部坐标系下检查是否在范围内
                    offset = hit_pt - board_center
                    local_y = np.dot(offset, board_y_dir)  # 宽度方向
                    local_z = np.dot(offset, board_z_dir)  # 高度方向
                    
                    if abs(local_y) <= BOARD_W / 2 and abs(local_z) <= BOARD_H / 2:
                        # 检查是否在圆孔内
                        in_hole = False
                        for cc in circle_centers:
                            dist = np.linalg.norm(hit_pt - cc)
                            if dist < CIRCLE_RADIUS:
                                in_hole = True
                                break
                        
                        if not in_hole:
                            points.append([hit_pt[0], hit_pt[1], hit_pt[2], LIDAR_INTENSITY])
                            hit_board = True
                            board_hits += 1
            
            # --- 没打到板面: 打到圆柱体背景 ---
            if not hit_board:
                # 圆柱体: x^2 + y^2 = R^2 (沿 Z 轴, 即垂直圆柱)
                a = dx**2 + dy**2
                if a > 1e-8:
                    t_cyl = CYLINDER_RADIUS / np.sqrt(a)
                    pt = ray_dir * t_cyl
                    points.append([pt[0], pt[1], pt[2], LIDAR_INTENSITY])

    points = np.array(points, dtype=np.float32)
    print(f"[点云] 生成了 {len(points)} 个点 (其中标定板: {board_hits} 个)")

    # --- 保存为 PCD 文件 ---
    os.makedirs(os.path.dirname(OUTPUT_PCD), exist_ok=True)
    write_pcd(OUTPUT_PCD, points)
    print(f"[点云] ✓ 已保存: {OUTPUT_PCD}")


def write_pcd(filepath, points):
    """以 ASCII 格式写入 PCD 文件. points: (N, 4) -> x, y, z, intensity."""
    n = len(points)
    header = f"""# .PCD v0.7 - Point Cloud Data file format
VERSION 0.7
FIELDS x y z intensity
SIZE 4 4 4 4
TYPE F F F F
COUNT 1 1 1 1
WIDTH {n}
HEIGHT 1
VIEWPOINT 0 0 0 1 0 0 0
POINTS {n}
DATA ascii
"""
    with open(filepath, 'w') as f:
        f.write(header)
        for p in points:
            f.write(f"{p[0]:.6f} {p[1]:.6f} {p[2]:.6f} {p[3]:.1f}\n")


# ============================================================
#                        主程序
# ============================================================

if __name__ == "__main__":
    print("=" * 50)
    print(" 生成 LiDAR-Camera 外参标定测试数据")
    print("=" * 50)
    
    # 打印已知外参
    print("\n已知外参 T_LiDAR->Camera:")
    print(T_LIDAR_TO_CAM)
    
    # 生成伪照片
    generate_image()
    
    # 生成伪点云
    generate_point_cloud()
    
    print("\n" + "=" * 50)
    print(f" 输出文件:")
    print(f"   图像: {os.path.abspath(OUTPUT_IMAGE)}")
    print(f"   点云: {os.path.abspath(OUTPUT_PCD)}")
    print(f"\n 已知真值外参 (用于验证标定结果):")
    print(f"   平移: tx=0.0, ty=-0.15, tz=0.0")
    print(f"   旋转: 雷达X->相机Z, 雷达Y->相机-X, 雷达Z->相机-Y")
    print("=" * 50)
