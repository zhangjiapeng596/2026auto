#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""地图可视化标点工具：直接在 PGM 图片上点击标注，自动换算 map 坐标。

用法 (Windows 端直接运行):
  python tools/mark_map_gui.py
  python tools/mark_map_gui.py -m game
  python tools/mark_map_gui.py -m game -o my_points.yaml

操作:
  左键点击第1次    = 添加点，进入朝向模式
  移动鼠标         = 调整车头方向
  左键点击第2次    = 确认朝向
  右键/ESC          = 取消朝向模式(朝向归0)
  双击点            = 重新调整朝向
  鼠标滚轮          = 缩放 (以光标为中心)
  中键/Shift+左键拖拽 = 平移
  R 键            = 重置视图
  输入名称 + 回车  = 为最新点命名
  S 键            = 保存 YAML
  ESC             = 退出

依赖: pip install pillow pyyaml
"""

import os
import sys
import math
import argparse
import yaml
import json
import tkinter as tk
from tkinter import ttk, messagebox
from datetime import datetime

try:
    from PIL import Image, ImageTk
except ImportError:
    print('Missing dependency: pillow')
    print('Install: pip install pillow pyyaml')
    sys.exit(1)


class ZoomPanCanvas(tk.Canvas):
    """可缩放/平移的 Canvas 封装。"""

    def __init__(self, parent, img_width, img_height, **kwargs):
        super().__init__(parent, **kwargs)
        self.img_w = img_width
        self.img_h = img_height
        self.zoom = 1.0
        self.pan_x = 0.0
        self.pan_y = 0.0

        # 拖拽状态
        self._drag_start_x = 0
        self._drag_start_y = 0
        self._drag_pan_x = 0.0
        self._drag_pan_y = 0.0
        self._dragging = False
        self.on_transform = None  # 缩放/平移后回调

        # 绑定
        self.bind('<MouseWheel>', self._on_mousewheel)       # Windows
        self.bind('<Button-4>', self._on_mousewheel_up)      # Linux scroll up
        self.bind('<Button-5>', self._on_mousewheel_down)    # Linux scroll down
        self.bind('<Button-2>', self._on_pan_start)           # 中键按下
        self.bind('<B2-Motion>', self._on_pan_move)           # 中键拖拽
        self.bind('<ButtonRelease-2>', self._on_pan_stop)
        self.bind('<Shift-Button-1>', self._on_pan_start)     # Shift+左键
        self.bind('<Shift-B1-Motion>', self._on_pan_move)
        self.bind('<Shift-ButtonRelease-1>', self._on_pan_stop)

    def reset_view(self):
        """重置缩放和平移到初始状态。"""
        cw = self.winfo_width()
        ch = self.winfo_height()
        self.zoom = min(cw / self.img_w, ch / self.img_h, 1.0)
        self.pan_x = (cw - self.img_w * self.zoom) / 2.0
        self.pan_y = (ch - self.img_h * self.zoom) / 2.0

    # ---- 坐标变换 ----
    def canvas_to_img(self, cx, cy):
        """Canvas 坐标 -> 原图像素坐标"""
        px = (cx - self.pan_x) / self.zoom
        py = (cy - self.pan_y) / self.zoom
        return px, py

    def img_to_canvas(self, px, py):
        """原图像素坐标 -> Canvas 坐标"""
        cx = px * self.zoom + self.pan_x
        cy = py * self.zoom + self.pan_y
        return cx, cy

    # ---- 缩放 ----
    def _on_mousewheel(self, event):
        """Windows 鼠标滚轮。"""
        self._zoom_at(event.x, event.y, 1.1 if event.delta > 0 else 0.9)

    def _on_mousewheel_up(self, event):
        self._zoom_at(event.x, event.y, 1.1)

    def _on_mousewheel_down(self, event):
        self._zoom_at(event.x, event.y, 0.9)

    def _zoom_at(self, cx, cy, factor):
        """以画布坐标 (cx, cy) 为中心缩放。"""
        new_zoom = self.zoom * factor
        new_zoom = max(0.1, min(new_zoom, 20.0))  # 限制范围
        factor = new_zoom / self.zoom

        self.pan_x = cx - (cx - self.pan_x) * factor
        self.pan_y = cy - (cy - self.pan_y) * factor
        self.zoom = new_zoom
        if self.on_transform:
            self.on_transform()

    # ---- 平移 ----
    def _on_pan_start(self, event):
        self._drag_start_x = event.x
        self._drag_start_y = event.y
        self._drag_pan_x = self.pan_x
        self._drag_pan_y = self.pan_y
        self._dragging = True
        self.config(cursor='fleur')

    def _on_pan_move(self, event):
        if not self._dragging:
            return
        self.pan_x = self._drag_pan_x + (event.x - self._drag_start_x)
        self.pan_y = self._drag_pan_y + (event.y - self._drag_start_y)

    def _on_pan_stop(self, event):
        self._dragging = False
        self.config(cursor='crosshair')
        if self.on_transform:
            self.on_transform()


class MapMarker(tk.Tk):
    """在地图 PGM 上交互式标点。"""

    def __init__(self, map_name='game', output_file=None):
        super().__init__()

        repo_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        maps_dir = os.path.join(repo_dir, 'src', 'robot_slam', 'maps')

        # 加载 YAML
        yaml_path = os.path.join(maps_dir, map_name + '.yaml')
        if not os.path.isfile(yaml_path):
            raise IOError('Map YAML not found: %s' % yaml_path)
        with open(yaml_path, 'r') as f:
            self.meta = yaml.safe_load(f)

        pgm_basename = self.meta.get('image', map_name + '.pgm')
        pgm_path = os.path.join(maps_dir, pgm_basename if not os.path.isabs(pgm_basename) else '')
        if not os.path.isabs(pgm_basename):
            pgm_path = os.path.join(maps_dir, pgm_basename)
        else:
            pgm_path = pgm_basename
        if not os.path.isfile(pgm_path):
            raise IOError('Map PGM not found: %s' % pgm_path)

        self.resolution = float(self.meta['resolution'])
        self.origin_x = float(self.meta['origin'][0])
        self.origin_y = float(self.meta['origin'][1])
        self.map_name = map_name

        if output_file is None:
            ts = datetime.now().strftime('%Y%m%d_%H%M%S')
            output_file = 'marked_%s_%s.yaml' % (map_name, ts)
        self.output_file = os.path.join(maps_dir, output_file)

        # 加载图片
        self.pil_img_orig = Image.open(pgm_path)
        self.img_w, self.img_h = self.pil_img_orig.size

        # 数据：存储原图像素坐标 + map 坐标 + 朝向
        # points: [(img_px, img_py, map_x, map_y, name, yaw_rad), ...]
        self.points = []
        self.counter = 0
        self.dot_radius = 3
        self.arrow_len = 14  # 箭头像素长度
        self._heading_mode = False  # 是否在等待用户拖拽确认朝向
        self._heading_point = None  # (ipx, ipy, mx, my) 等待确认朝向的点
        self._heading_line = None   # canvas line id for preview

        # ---- UI ----
        self.title('Map Marker: %s  (%.3f m/px | %dx%d px | origin [%.2f, %.2f])' % (
            map_name, self.resolution, self.img_w, self.img_h,
            self.origin_x, self.origin_y))
        self.geometry('1400x950')
        self.minsize(800, 600)

        # 工具栏
        toolbar = ttk.Frame(self)
        toolbar.pack(side=tk.TOP, fill=tk.X, padx=5, pady=4)

        ttk.Label(toolbar, text='名称:').pack(side=tk.LEFT, padx=(0, 3))
        self.name_var = tk.StringVar()
        self.name_entry = ttk.Entry(toolbar, textvariable=self.name_var, width=14)
        self.name_entry.pack(side=tk.LEFT, padx=(0, 4))
        self.name_entry.bind('<Return>', lambda e: self._rename_last())

        ttk.Button(toolbar, text='重命名', command=self._rename_last).pack(side=tk.LEFT, padx=2)
        ttk.Button(toolbar, text='撤销', command=self._undo).pack(side=tk.LEFT, padx=2)
        ttk.Button(toolbar, text='清空', command=self._clear).pack(side=tk.LEFT, padx=2)
        ttk.Separator(toolbar, orient=tk.VERTICAL).pack(side=tk.LEFT, fill=tk.Y, padx=6)
        ttk.Button(toolbar, text='保存 YAML', command=self._save).pack(side=tk.LEFT, padx=2)
        ttk.Button(toolbar, text='点列表', command=self._show_list).pack(side=tk.LEFT, padx=2)
        ttk.Separator(toolbar, orient=tk.VERTICAL).pack(side=tk.LEFT, fill=tk.Y, padx=6)
        ttk.Button(toolbar, text='重置视图 (R)', command=self._reset_view).pack(side=tk.LEFT, padx=2)

        zoom_label = ttk.Label(toolbar, text='  滚轮缩放 | Shift+左键平移 | 中键平移')
        zoom_label.pack(side=tk.RIGHT, padx=5)

        # 状态栏
        self.status_var = tk.StringVar(
            value='滚轮=缩放  Shift+拖拽=平移  R=重置  左键=加标点  S=保存')
        status = ttk.Label(self, textvariable=self.status_var, relief=tk.SUNKEN, anchor=tk.W)
        status.pack(side=tk.BOTTOM, fill=tk.X)

        # 可缩放画布
        self.canvas = ZoomPanCanvas(self, self.img_w, self.img_h,
                                     cursor='crosshair', bg='#2b2b2b',
                                     highlightthickness=0)
        self.canvas.on_transform = self._on_view_changed
        self.canvas.pack(fill=tk.BOTH, expand=True)

        # 画布初始化后加载图片并重置视图
        self.canvas.bind('<Configure>', self._on_canvas_configure, add='+')
        self._img_loaded = False
        self._load_image()

        # 点图层
        self._dot_ovals = []
        self._dot_labels = []

        # 键盘事件
        self.bind('<KeyPress-s>', lambda e: self._save())
        self.bind('<KeyPress-S>', lambda e: self._save())
        self.bind('<KeyPress-r>', lambda e: self._reset_view())
        self.bind('<KeyPress-R>', lambda e: self._reset_view())
        self.bind('<Control-z>', lambda e: self._undo())
        self.bind('<Escape>', lambda e: self.destroy())

        # 左键/右键点击（画布级）
        self.canvas.bind('<Button-1>', self._on_left_click)
        self.canvas.bind('<Button-3>', self._on_right_click)
        self.canvas.bind('<Double-Button-1>', self._on_double_click)
        self.canvas.bind('<Motion>', self._on_mouse_move)
        self.bind('<Escape>', self._on_escape)

        self.protocol('WM_DELETE_WINDOW', self._on_close)

        # 打印信息
        print('')
        print('=' * 60)
        print('  地图标点工具 — %s' % map_name)
        print('  分辨率: %.4f m/px | 图片: %d×%d px' % (self.resolution, self.img_w, self.img_h))
        print('  地图范围: x=[%.2f, %.2f]  y=[%.2f, %.2f]' % (
            self.origin_x, self.origin_x + self.img_w * self.resolution,
            self.origin_y, self.origin_y + self.img_h * self.resolution))
        print('')
        print('  滚轮=缩放  Shift+拖拽=平移  中键拖拽=平移  R=重置视图')
        print('  左键第1次=放置点(进入朝向模式)  移动鼠标=设车头方向  左键第2次=确认')
        print('  双击点=重新调朝向  右键=取消朝向/撤销  S=保存  ESC=退出')
        print('=' * 60)

    def _load_image(self):
        """渲染原图到 Canvas（作为背景）。"""
        # 用原始图片，通过缩放和平移显示
        self._tk_img = ImageTk.PhotoImage(self.pil_img_orig)
        self._bg_img_id = self.canvas.create_image(0, 0, anchor=tk.NW,
                                                     image=self._tk_img, tags=('bg',))
        self._img_loaded = True

    def _on_canvas_configure(self, event):
        """窗口尺寸变化时重置视图。"""
        if self._img_loaded and event.width > 10 and event.height > 10:
            self.canvas.reset_view()
            self._update_bg_transform()
            self._redraw_points()

    def _update_bg_transform(self):
        """更新背景图片的缩放/平移。"""
        scale_x = max(1, int(self.img_w * self.canvas.zoom))
        scale_y = max(1, int(self.img_h * self.canvas.zoom))
        self.canvas.coords(self._bg_img_id, self.canvas.pan_x, self.canvas.pan_y)
        self._tk_img = ImageTk.PhotoImage(
            self.pil_img_orig.resize((scale_x, scale_y), Image.NEAREST))
        self.canvas.itemconfig(self._bg_img_id, image=self._tk_img)

    def _reset_view(self):
        self.canvas.reset_view()
        self._update_bg_transform()
        self._redraw_points()
        self.status_var.set('视图已重置  zoom=%.2fx' % self.canvas.zoom)

    def _on_view_changed(self):
        """缩放/平移后更新背景图和点。"""
        self._update_bg_transform()
        self._redraw_points()

    # ---- 坐标换算 ----
    def _img_to_map(self, px, py):
        """原图像素坐标 -> map 坐标"""
        mx = px * self.resolution + self.origin_x
        my = (self.img_h - py) * self.resolution + self.origin_y
        return round(mx, 4), round(my, 4)

    def _map_to_img(self, mx, my):
        """map 坐标 -> 原图像素坐标"""
        px = (mx - self.origin_x) / self.resolution
        py = self.img_h - (my - self.origin_y) / self.resolution
        return px, py

    # ---- 事件 ----
    def _on_left_click(self, event):
        """左键：第1次放点进入朝向模式，第2次确认朝向。"""
        ipx, ipy = self.canvas.canvas_to_img(event.x, event.y)
        if not (0 <= ipx < self.img_w and 0 <= ipy < self.img_h):
            return

        if self._heading_mode:
            # 第2次点击：确认朝向
            mx, my = self._heading_point[2], self._heading_point[3]
            # 计算 yaw: atan2(dy, dx) where dx,dy is from point to cursor in map frame
            # 图像坐标 y 轴向下, map 坐标 y 轴向上
            dy_img = ipy - self._heading_point[1]  # 图像坐标差
            dx_img = ipx - self._heading_point[0]
            yaw = math.atan2(-dy_img, dx_img)  # 翻转Y得到map朝向
            self.points.append((self._heading_point[0], self._heading_point[1],
                               self._heading_point[2], self._heading_point[3],
                               self._heading_point[4], yaw))
            self._heading_mode = False
            self._heading_point = None
            self._clear_heading_preview()
            self._redraw_points()
            self.status_var.set('确认点 %s  yaw=%.2f rad (%.1f°)' %
                               (self.points[-1][4], yaw, math.degrees(yaw)))
            print('[+] %s: map=(%.4f, %.4f)  yaw=%.2f rad (%.1f°)' %
                  (self.points[-1][4], mx, my, yaw, math.degrees(yaw)))
        else:
            # 第1次点击：放置点，进入朝向模式
            self.counter += 1
            mx, my = self._img_to_map(ipx, ipy)
            name = 'P%d' % self.counter
            self._heading_mode = True
            self._heading_point = (ipx, ipy, mx, my, name)
            self.status_var.set('放置 %s  map=(%.4f,%.4f)  移动鼠标设置朝向, 再次点击确认, 右键/Esc取消' %
                               (name, mx, my))

    def _on_double_click(self, event):
        """双击已有点：重新进入朝向模式。"""
        ipx, ipy = self.canvas.canvas_to_img(event.x, event.y)
        # 找最近的已有点
        if not self.points:
            return
        closest = None
        closest_dist = float('inf')
        for i, (px, py, mx, my, name, yaw) in enumerate(self.points):
            dist = math.hypot(ipx - px, ipy - py)
            if dist < closest_dist:
                closest_dist = dist
                closest = i
        threshold = 15 / self.canvas.zoom  # 像素距离阈值
        if closest is not None and closest_dist < threshold:
            px, py, mx, my, name, yaw = self.points.pop(closest)
            self._heading_mode = True
            self._heading_point = (px, py, mx, my, name)
            self._redraw_points()
            self.status_var.set('正在调整 %s 的朝向, 点击确认, 右键/Esc取消' % name)

    def _on_right_click(self, event):
        """右键：朝向模式下取消朝向(设yaw=0)，普通模式下撤销最近点。"""
        if self._heading_mode:
            # 取消朝向模式，以 yaw=0 确认
            px, py, mx, my, name = self._heading_point
            self.points.append((px, py, mx, my, name, 0.0))
            self._heading_mode = False
            self._heading_point = None
            self._clear_heading_preview()
            self._redraw_points()
            self.status_var.set('取消朝向 %s  yaw=0 (朝向归零)' % name)
            print('[+] %s: map=(%.4f, %.4f)  yaw=0 (no heading)' % (name, mx, my))
        else:
            self._undo()

    def _on_escape(self, event):
        """Esc: 取消朝向模式。"""
        if self._heading_mode:
            self._on_right_click(None)

    def _on_mouse_move(self, event):
        ipx, ipy = self.canvas.canvas_to_img(event.x, event.y)
        in_img = 0 <= ipx < self.img_w and 0 <= ipy < self.img_h

        # 朝向模式下更新预览箭头
        if self._heading_mode and in_img:
            px, py = self._heading_point[0], self._heading_point[1]
            dy_img = ipy - py
            dx_img = ipx - px
            yaw = math.atan2(-dy_img, dx_img)
            self._draw_heading_preview(px, py, yaw)

        if in_img:
            mx, my = self._img_to_map(ipx, ipy)
            self.status_var.set(
                '像素:(%.1f, %.1f)  map:(%.4f, %.4f)  |  已标 %d 点  zoom=%.1fx  |  %s' %
                (ipx, ipy, mx, my, len(self.points), self.canvas.zoom,
                 '【朝向模式：移动鼠标→点击确认】' if self._heading_mode else '左键标点'))
        else:
            self.status_var.set('鼠标在地图外  |  已标 %d 点  zoom=%.1fx' % (len(self.points), self.canvas.zoom))

    def _draw_heading_preview(self, ipx, ipy, yaw):
        """绘制朝向预览线。"""
        self._clear_heading_preview()
        cx, cy = self.canvas.img_to_canvas(ipx, ipy)
        z = self.canvas.zoom
        al = self.arrow_len * z
        end_cx = cx + math.cos(yaw) * al
        end_cy = cy - math.sin(yaw) * al  # Canvas Y向下
        self._heading_line = self.canvas.create_line(
            cx, cy, end_cx, end_cy,
            fill='#ff4444', width=max(2, int(3 * z)),
            arrow=tk.LAST, arrowshape=(int(10 * z), int(12 * z), int(5 * z)),
            tags=('heading_preview',))

    def _clear_heading_preview(self):
        if self._heading_line:
            self.canvas.delete(self._heading_line)
            self._heading_line = None

    def _on_close(self):
        if self.points:
            if messagebox.askyesno('保存', '退出前保存 %d 个点到 YAML？' % len(self.points)):
                self._save()
        self.destroy()

    # ---- 点操作（坐标都是原图像素） ----
    def _clear_canvas_dots(self):
        for o in self._dot_ovals:
            self.canvas.delete(o)
        for t in self._dot_labels:
            self.canvas.delete(t)
        self._dot_ovals = []
        self._dot_labels = []

    def _redraw_points(self):
        self._clear_canvas_dots()
        z = self.canvas.zoom
        r = max(2, int(self.dot_radius * z))  # 圆点大小随缩放
        font_size = max(7, int(10 * z))
        al = self.arrow_len * z  # 箭头长度
        for ipx, ipy, mx, my, name, yaw in self.points:
            cx, cy = self.canvas.img_to_canvas(ipx, ipy)
            # 圆点
            oval = self.canvas.create_oval(cx - r, cy - r, cx + r, cy + r,
                                            fill='#ff4444', outline='#ffff00',
                                            width=max(1, int(1.5 * z)),
                                            tags=('dot',))
            self._dot_ovals.append(oval)
            # 标签
            txt = self.canvas.create_text(cx + r + 2, cy - r - 2,
                                           text=name, anchor=tk.NW,
                                           fill='#00ff88',
                                           font=('Consolas', font_size, 'bold'),
                                           tags=('label',))
            self._dot_labels.append(txt)
            # 朝向箭头 (canvas: Y向下)
            end_cx = cx + math.cos(yaw) * al
            end_cy = cy - math.sin(yaw) * al
            arrow = self.canvas.create_line(
                cx, cy, end_cx, end_cy,
                fill='#ffaa00', width=max(1, int(2 * z)),
                arrow=tk.LAST, arrowshape=(int(8 * z), int(10 * z), int(4 * z)),
                tags=('arrow',))
            self._dot_ovals.append(arrow)

    def _rename_last(self):
        new_name = self.name_var.get().strip()
        if not new_name:
            self.status_var.set('请先在名称框输入新名字')
            return
        if not self.points:
            self.status_var.set('没有点可重命名')
            return
        ipx, ipy, mx, my, _, yaw = self.points[-1]
        self.points[-1] = (ipx, ipy, mx, my, new_name, yaw)
        self._redraw_points()
        self.name_var.set('')
        self.status_var.set('已重命名为 "%s"' % new_name)
        print('[*] Renamed last -> "%s"' % new_name)

    def _undo(self):
        if self.points:
            removed = self.points.pop()
            self._redraw_points()
            self.status_var.set('撤销 %s (剩余 %d 点)' % (removed[4], len(self.points)))
            print('[-] Removed %s, %d remaining' % (removed[4], len(self.points)))
        else:
            self.status_var.set('没有点可撤销')

    def _clear(self):
        if self.points and messagebox.askyesno('确认', '清空全部 %d 个点？' % len(self.points)):
            self.points = []
            self.counter = 0
            self._redraw_points()
            self.status_var.set('已清空全部点')

    def _save(self):
        if not self.points:
            self.status_var.set('没有点可保存!')
            return

        data = {
            'description': 'Points marked on %s map using mark_map_gui.py' % self.map_name,
            'timestamp': datetime.now().isoformat(),
            'map': self.map_name,
            'resolution': self.resolution,
            'origin': [self.origin_x, self.origin_y,
                       self.meta['origin'][2] if len(self.meta['origin']) > 2 else 0.0],
            'count': len(self.points),
            'points': [],
        }
        for ipx, ipy, mx, my, name, yaw in self.points:
            data['points'].append({'name': name, 'x': mx, 'y': my, 'z': 0.0,
                                   'yaw_rad': round(yaw, 4), 'yaw_deg': round(math.degrees(yaw), 1)})

        yaml_path = self.output_file
        json_path = yaml_path.replace('.yaml', '.json').replace('.yml', '.json')

        with open(yaml_path, 'w') as f:
            yaml.dump(data, f, default_flow_style=False, allow_unicode=True)
        with open(json_path, 'w') as f:
            json.dump(data, f, indent=2, ensure_ascii=False)

        self.status_var.set('已保存 %d 个点 -> %s' % (len(self.points), yaml_path))
        print('')
        print('=' * 60)
        print('  Saved: %s' % yaml_path)
        for p in data['points']:
            print('    %s: [%.4f, %.4f, 0.0]  yaw: %.1f°' %
                  (p['name'], p['x'], p['y'], p.get('yaw_deg', 0)))
        print('=' * 60)

    def _show_list(self):
        if not self.points:
            messagebox.showinfo('点列表', '还没有标记任何点')
            return
        win = tk.Toplevel(self)
        win.title('已标记的点 (%d)' % len(self.points))
        win.geometry('560x400')
        frame = ttk.Frame(win)
        frame.pack(fill=tk.BOTH, expand=True, padx=8, pady=8)
        ttk.Label(frame, text='%-4s  %-14s  %12s  %12s  %s' % ('#', 'Name', 'x', 'y', 'z'),
                  font=('Consolas', 10, 'bold')).pack(anchor=tk.W, pady=2)
        text = tk.Text(frame, font=('Consolas', 10))
        text.pack(fill=tk.BOTH, expand=True)
        for i, (ipx, ipy, mx, my, name, yaw) in enumerate(self.points):
            text.insert(tk.END, '%-4d  %-14s  %12.4f  %12.4f  0.0  yaw: %.1f°\n' %
                       (i + 1, name, mx, my, math.degrees(yaw)))
        text.config(state=tk.DISABLED)

        def _copy():
            lines = ['%s: [%.4f, %.4f, 0.0]  yaw: %.2f rad (%.1f°)' %
                     (n, x, y, yw, math.degrees(yw)) for _, _, x, y, n, yw in self.points]
            self.clipboard_clear()
            self.clipboard_append('\n'.join(lines))
            self.status_var.set('已复制 %d 个点到剪贴板' % len(self.points))

        ttk.Button(win, text='复制到剪贴板', command=_copy).pack(pady=5)


def main():
    parser = argparse.ArgumentParser(description='PGM 地图可视化标点工具（支持缩放平移）')
    parser.add_argument('-m', '--map', default='game', help='地图名 (默认: game)')
    parser.add_argument('-o', '--output', default=None, help='输出 YAML 文件名')
    args = parser.parse_args()
    app = MapMarker(map_name=args.map, output_file=args.output)
    app.mainloop()


if __name__ == '__main__':
    main()
