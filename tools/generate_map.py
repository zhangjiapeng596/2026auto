#!/usr/bin/env python
"""生成比赛场地 PGM 地图。从 YAML 配置读取参数，生成带围栏和障碍物的地图。

用法:
  python tools/generate_map.py                           # 默认配置
  python tools/generate_map.py -o my_map                 # 指定输出名
  python tools/generate_map.py -c config/competition_field.yaml  # 指定配置
"""
import os
import sys
import math
import argparse
import yaml

try:
    from PIL import Image, ImageDraw
except ImportError:
    print('pip install pillow pyyaml')
    sys.exit(1)

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def generate_map(config_path, output_name):
    with open(config_path, 'r', encoding='utf-8') as f:
        cfg = yaml.safe_load(f)

    field = cfg['field']
    field_w = field['size_m'][0]  # 3.6
    field_h = field['size_m'][1]  # 3.6
    grid_rows = field['grid_rows']  # 9
    grid_cols = field['grid_cols']  # 9
    cell_sz = field['cell_size_m']  # 0.4
    fence_h = field.get('fence_height_m', 0.30)

    # 地图参数
    margin = 0.6  # 场地外围边距 (m) — 给导航留缓冲
    resolution = 0.02  # m/pixel
    map_w = field_w + 2 * margin
    map_h = field_h + 2 * margin
    img_w = int(map_w / resolution)
    img_h = int(map_h / resolution)
    origin_x = -map_w / 2.0
    origin_y = -map_h / 2.0

    # 创建白色画布 (254 = 自由空间)
    img = Image.new('L', (img_w, img_h), 254)
    draw = ImageDraw.Draw(img)

    def world_to_pixel(mx, my):
        """map 坐标 → 像素坐标 (左上角原点, Y向下)"""
        px = int(round((mx - origin_x) / resolution))
        py = int(round(img_h - (my - origin_y) / resolution))
        return px, py

    # ---- 围栏 (黑色边框) ----
    fence_l = world_to_pixel(-field_w / 2, 0)[0]
    fence_r = world_to_pixel(field_w / 2, 0)[0]
    fence_t = world_to_pixel(0, field_h / 2)[1]
    fence_b = world_to_pixel(0, -field_h / 2)[1]
    # 围栏: 1 像素细线 (仿真不需要物理厚度, 否则起点在围栏里)
    draw.rectangle([fence_l, fence_t, fence_r, fence_b], outline=0, width=2)

    # ---- 网格线 (浅灰, 不影响导航) ----
    grid_gray = 200
    for r in range(1, grid_rows):
        y = field_h / 2 - r * cell_sz
        x1, _ = world_to_pixel(-field_w / 2, y)
        x2, _ = world_to_pixel(field_w / 2, y)
        _, py = world_to_pixel(0, y)
        draw.line([(x1, py), (x2, py)], fill=grid_gray, width=1)
    for c in range(1, grid_cols):
        x = -field_w / 2 + c * cell_sz
        _, py1 = world_to_pixel(x, field_h / 2)
        _, py2 = world_to_pixel(x, -field_h / 2)
        px, _ = world_to_pixel(x, 0)
        draw.line([(px, py1), (px, py2)], fill=grid_gray, width=1)

    # ---- 障碍物 (由 sim_robot 动态注入激光数据, 不写入 PGM) ----
    obstacles = cfg.get('obstacles') or []
    if obstacles:
        print('[i] %d obstacle(s) on internal grid edges (injected via sim_robot LiDAR)' % len(obstacles))
        for obs in obstacles:
            cx, cy, yaw = obstacle_pose(obs, grid_rows, grid_cols, cell_sz, field_w, field_h)
            print('    cell %d edge %s: center=(%.3f, %.3f), yaw=%.1fdeg' %
                  (obs['cell'], str(obs.get('edge')).upper(), cx, cy, math.degrees(yaw)))

    # ---- 保存 ----
    maps_dir = os.path.join(REPO, 'src', 'robot_slam', 'maps')
    pgm_path = os.path.join(maps_dir, output_name + '.pgm')
    yaml_path = os.path.join(maps_dir, output_name + '.yaml')

    img.save(pgm_path)
    print('[+] PGM saved: %s (%dx%d px, %.3f m/px)' %
          (pgm_path, img_w, img_h, resolution))

    map_yaml = {
        'image': output_name + '.pgm',
        'resolution': resolution,
        'origin': [origin_x, origin_y, 0.0],
        'negate': 0,
        'occupied_thresh': 0.65,
        'free_thresh': 0.196,
    }
    with open(yaml_path, 'w') as f:
        yaml.dump(map_yaml, f, default_flow_style=None)
    print('[+] YAML saved: %s' % yaml_path)

    # ---- 打印关键坐标 ----
    print('')
    print('=== 坐标参考 (map坐标系, 原点在场地中心) ===')
    print('场地范围: x=[%.2f, %.2f]  y=[%.2f, %.2f]' %
          (-field_w / 2, field_w / 2, -field_h / 2, field_h / 2))
    print('起点 cell 1:  (%.3f, %.3f)' % cell_center(1, grid_cols, cell_sz, field_w, field_h))
    print('终点 cell 9:  (%.3f, %.3f)' % cell_center(9, grid_cols, cell_sz, field_w, field_h))
    print('起点 cell 81: (%.3f, %.3f)' % cell_center(81, grid_cols, cell_sz, field_w, field_h))

    # 视觉点位
    vp = cfg.get('vision_positions', [])
    if vp:
        print('视觉点位:')
        for v in vp:
            cx, cy = cell_center(v, grid_cols, cell_sz, field_w, field_h)
            print('  cell %d: (%.3f, %.3f)' % (v, cx, cy))

    # 障碍物
    if obstacles:
        print('障碍物:')
        for obs in obstacles:
            cx, cy, yaw = obstacle_pose(obs, grid_rows, grid_cols, cell_sz, field_w, field_h)
            print('  cell %d edge %s: (%.3f, %.3f), yaw=%.1fdeg' %
                  (obs['cell'], str(obs.get('edge')).upper(), cx, cy, math.degrees(yaw)))


def cell_center(cell_number, cols, cell_sz, field_w, field_h):
    """网格编号 → map 坐标中心 (编号约定同 competition_field.yaml)"""
    n = cell_number - 1
    row = n // cols
    col = n % cols
    x = (col - cols / 2.0) * cell_sz + cell_sz / 2.0
    rows = int(round(field_h / cell_sz))
    y = (rows / 2.0 - row) * cell_sz - cell_sz / 2.0
    return round(x, 4), round(y, 4)


def board_dimensions(cfg):
    """返回挡板的 (length, thickness)。"""
    size = cfg.get('obstacles_config', {}).get('board_size_m', [0.01, 0.40])
    if not isinstance(size, list) or len(size) < 2:
        size = [0.01, 0.40]
    thickness = min(float(size[0]), float(size[1]))
    length = max(float(size[0]), float(size[1]))
    return length, thickness


def obstacle_pose(obs, rows, cols, cell_sz, field_w, field_h):
    """把 {cell, edge} 转成内部网格边上的挡板中心和朝向。"""
    cell = obs['cell']
    if cell < 1 or cell > rows * cols:
        raise ValueError('obstacle cell %s is outside 1..%d' % (cell, rows * cols))

    n = cell - 1
    row = n // cols
    col = n % cols
    edge = str(obs.get('edge', '')).upper()
    if edge not in ('N', 'S', 'E', 'W'):
        raise ValueError('obstacle cell %s must set edge=N/S/E/W' % cell)
    if ((edge == 'N' and row == 0) or
            (edge == 'S' and row == rows - 1) or
            (edge == 'W' and col == 0) or
            (edge == 'E' and col == cols - 1)):
        raise ValueError('obstacle cell %s edge %s is outer boundary' % (cell, edge))

    cx, cy = cell_center(cell, cols, cell_sz, field_w, field_h)
    if edge == 'N':
        cy += cell_sz / 2.0
        yaw = 0.0
    elif edge == 'S':
        cy -= cell_sz / 2.0
        yaw = 0.0
    elif edge == 'E':
        cx += cell_sz / 2.0
        yaw = math.pi / 2.0
    else:
        cx -= cell_sz / 2.0
        yaw = math.pi / 2.0
    return cx, cy, yaw


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='生成比赛场地 PGM 地图')
    parser.add_argument('-c', '--config', default='competition_field',
                        help='配置文件名 (默认: competition_field)')
    parser.add_argument('-o', '--output', default=None,
                        help='输出地图名 (默认: 同配置名)')
    args = parser.parse_args()

    config_name = args.config.replace('.yaml', '')
    config_path = os.path.join(REPO, 'config', config_name + '.yaml')
    if not os.path.isfile(config_path):
        print('Config not found: %s' % config_path)
        sys.exit(1)

    output_name = args.output or config_name
    generate_map(config_path, output_name)
