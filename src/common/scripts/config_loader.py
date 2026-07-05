#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""配置加载工具：从 config/ YAML 文件加载比赛参数。"""
import os
import sys
reload(sys)
sys.setdefaultencoding('utf-8')
import yaml
import rospy

CONFIG_SEARCH_PATHS = [
    os.path.join(os.path.dirname(__file__), '..', '..', '..', 'config'),
    os.path.join(os.path.expanduser('~'), '3X2KY2_backup2', 'config'),
    os.path.join(os.environ.get('WS_PATH', ''), 'config'),
]


def _find_config_dir():
    for path in CONFIG_SEARCH_PATHS:
        abs_path = os.path.abspath(path)
        if os.path.isdir(abs_path):
            return abs_path
    raise IOError('Cannot find config directory. Searched: {}'.format(CONFIG_SEARCH_PATHS))


def load_config(filename):
    """加载一个 YAML 配置文件，返回 dict。"""
    config_dir = _find_config_dir()
    filepath = os.path.join(config_dir, filename)
    if not os.path.isfile(filepath):
        raise IOError('Config file not found: {}'.format(filepath))
    with open(filepath, 'r') as f:
        data = yaml.safe_load(f)
    rospy.loginfo('[config_loader] Loaded: %s', filepath)
    return data


def load_all_configs():
    """加载全部 6 个配置文件，返回合并后的 dict。"""
    files = [
        'competition_field.yaml',
        'mission.yaml',
        'robot.yaml',
        'navigation.yaml',
        'perception.yaml',
        'voice_text.yaml',
    ]
    configs = {}
    for f in files:
        configs[f.replace('.yaml', '')] = load_config(f)
    return configs


def get_cell_center_xy(cell_number, field_config):
    """
    将网格编号 (1-81) 转换为 map 坐标系的 (x, y) 中心点。
    网格 1 在左上角，9 在右上角。
    map 坐标系原点在场地中心，x 轴向东，y 轴向北。
    """
    gc = field_config['cell_index_convention']
    rows = field_config['field']['grid_rows']
    cols = field_config['field']['grid_cols']
    cell_size = field_config['field']['cell_size_m']

    n = cell_number - 1
    row = n // cols  # 0 为最北
    col = n % cols   # 0 为最西

    x = (col - cols / 2.0) * cell_size + cell_size / 2.0
    y = (rows / 2.0 - row) * cell_size - cell_size / 2.0
    return x, y


def _point_in_polygon(px, py, polygon):
    """射线法判断点是否在多边形内部。polygon 为 [(x,y), ...] 顶点列表。"""
    n = len(polygon)
    inside = False
    j = n - 1
    for i in range(n):
        xi, yi = polygon[i]
        xj, yj = polygon[j]
        # 跳过水平边或退化边，防止除零（yj == yi）
        if abs(yj - yi) < 1e-12:
            j = i
            continue
        if ((yi > py) != (yj > py)) and (px < (xj - xi) * (py - yi) / (yj - yi) + xi):
            inside = not inside
        j = i
    return inside


def check_footprint_in_region(robot_x, robot_y, robot_yaw, footprint, cell_number, field_config):
    """
    判断机器人 footprint 是否完全进入指定任务点区域。

    Args:
        robot_x, robot_y: 机器人 base_link 在 map 坐标系下的位置 (m)
        robot_yaw: 机器人朝向角 (rad)
        footprint: 机器人 footprint 顶点列表 [[x1,y1], [x2,y2], ...]，相对于 base_link
        cell_number: 目标任务点网格编号 (1-81)
        field_config: 场地配置 dict（competition_field.yaml 的内容）

    Returns:
        (in_region, detail): in_region 为 True 表示完全进入，
                             detail 包含任务区域中心和超出点信息
    """
    import math

    # 获取任务点中心
    cx, cy = get_cell_center_xy(cell_number, field_config)
    task_w, task_h = field_config['task_region']['size_m']

    # 任务区域多边形的四个顶点（轴对齐矩形）
    half_w = task_w / 2.0
    half_h = task_h / 2.0
    region_polygon = [
        (cx - half_w, cy - half_h),
        (cx + half_w, cy - half_h),
        (cx + half_w, cy + half_h),
        (cx - half_w, cy + half_h),
    ]

    # 将 footprint 顶点从 base_link 坐标系变换到 map 坐标系
    cos_yaw = math.cos(robot_yaw)
    sin_yaw = math.sin(robot_yaw)
    points_outside = []

    for (fx, fy) in footprint:
        # 旋转 + 平移
        mx = cos_yaw * fx - sin_yaw * fy + robot_x
        my = sin_yaw * fx + cos_yaw * fy + robot_y
        if not _point_in_polygon(mx, my, region_polygon):
            points_outside.append((mx, my))

    in_region = len(points_outside) == 0
    detail = {
        'task_center': (cx, cy),
        'task_size': (task_w, task_h),
        'points_outside': points_outside,
    }
    return in_region, detail
