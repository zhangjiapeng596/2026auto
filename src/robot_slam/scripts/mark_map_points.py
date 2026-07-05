#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""地图标点工具：在 RViz 中用 "Publish Point" 点击采集坐标，保存为 YAML。

使用方法:
  1. roslaunch mission_manager sim_navigation.launch  # 启动地图+机器人
  2. rosrun robot_slam mark_map_points.py              # 启动标点工具
  3. 在 RViz 中点击 "Publish Point" 工具，点一次录一个
  4. 终端中输入点名称（回车跳过则自动编号）
  5. 输入 'save' 保存为 YAML，输入 'quit' 退出

依赖 topic: /clicked_point (geometry_msgs/PointStamped, RViz Publish Point)
"""

import os
import sys
reload(sys)
sys.setdefaultencoding('utf-8')

import rospy
import yaml
import threading
from geometry_msgs.msg import PointStamped
from datetime import datetime


class MapPointMarker(object):
    """监听 RViz Publish Point 点击，采集并保存地图坐标点。"""

    def __init__(self, output_dir=None):
        self.points = []  # list of dict: {name, x, y, z}
        self.last_click = None
        self.lock = threading.Lock()
        self.counter = 1

        # 输出目录默认为 robot_slam/maps/
        if output_dir is None:
            pkg_scripts = os.path.dirname(os.path.abspath(__file__))
            output_dir = os.path.join(pkg_scripts, '..', 'maps')
        self.output_dir = os.path.abspath(output_dir)
        if not os.path.isdir(self.output_dir):
            os.makedirs(self.output_dir)

        rospy.Subscriber('/clicked_point', PointStamped, self._on_click)

        rospy.loginfo('[Marker] Ready. Use RViz "Publish Point" tool to click on the map.')
        rospy.loginfo('[Marker] Commands: <name> = name the last point, save = save YAML, list = show all, undo = remove last, quit = exit')

    def _on_click(self, msg):
        """接收 RViz 点击。"""
        x = round(msg.point.x, 4)
        y = round(msg.point.y, 4)
        z = round(msg.point.z, 4)
        frame = msg.header.frame_id
        with self.lock:
            self.last_click = (x, y, z, frame)
        rospy.loginfo('[Marker] Clicked: (%.4f, %.4f, %.4f) in frame [%s]', x, y, z, frame)

    def add_point(self, name):
        """将最近一次点击加入列表。"""
        with self.lock:
            if self.last_click is None:
                rospy.logwarn('[Marker] No click recorded yet! Click on the map first.')
                return
            x, y, z, frame = self.last_click
            self.points.append({
                'name': name,
                'x': x,
                'y': y,
                'z': z,
                'frame': frame,
            })
            self.last_click = None
        rospy.loginfo('[Marker] Saved point #%d: "%s" = (%.4f, %.4f, %.4f)',
                      len(self.points), name, x, y, z)

    def add_point_auto(self):
        """自动编号添加最近一次点击。"""
        name = 'point_%d' % self.counter
        self.counter += 1
        self.add_point(name)

    def undo(self):
        """删除最近一个点。"""
        if self.points:
            removed = self.points.pop()
            rospy.loginfo('[Marker] Removed: "%s" (%.4f, %.4f)', removed['name'], removed['x'], removed['y'])
        else:
            rospy.loginfo('[Marker] No points to undo.')

    def list_points(self):
        """列出所有已采集的点。"""
        if not self.points:
            rospy.loginfo('[Marker] No points recorded yet.')
            return
        rospy.loginfo('[Marker] === Recorded Points (%d) ===', len(self.points))
        for i, p in enumerate(self.points):
            rospy.loginfo('[Marker]   %d. "%s": [%.4f, %.4f, %.4f]',
                          i + 1, p['name'], p['x'], p['y'], p['z'])

    def save(self, filename=None):
        """保存点到 YAML 文件。"""
        if not self.points:
            rospy.logwarn('[Marker] No points to save.')
            return

        if filename is None:
            ts = datetime.now().strftime('%Y%m%d_%H%M%S')
            filename = 'marked_points_%s.yaml' % ts

        filepath = os.path.join(self.output_dir, filename)

        output = {
            'description': 'Points marked on map using RViz Publish Point tool',
            'timestamp': datetime.now().isoformat(),
            'map': self._guess_map_name(),
            'count': len(self.points),
            'points': [],
        }
        for p in self.points:
            output['points'].append({
                'name': p['name'],
                'x': p['x'],
                'y': p['y'],
                'z': p['z'],
                'frame': p['frame'],
            })

        with open(filepath, 'w') as f:
            yaml.dump(output, f, default_flow_style=False, allow_unicode=True)

        rospy.loginfo('[Marker] Saved %d points to: %s', len(self.points), filepath)
        # 同时打印便于复制
        rospy.loginfo('[Marker] === Copy-ready format ===')
        for p in self.points:
            rospy.loginfo('[Marker]   %s: [%.4f, %.4f, %.4f]', p['name'], p['x'], p['y'], p['z'])

    def _guess_map_name(self):
        """尝试获取当前加载的地图名。"""
        try:
            params = rospy.get_param_names()
            for p in params:
                if 'map_server' in p:
                    return 'game'  # 默认
            return 'game'
        except Exception:
            return 'game'


def main():
    rospy.init_node('map_point_marker')

    output_dir = rospy.get_param('~output_dir', None)
    marker = MapPointMarker(output_dir)

    rospy.loginfo('')
    rospy.loginfo('=' * 60)
    rospy.loginfo('  地图标点工具')
    rospy.loginfo('  在 RViz 中点击 "Publish Point" 图标后在地图上点击')
    rospy.loginfo('  终端命令:')
    rospy.loginfo('    <回车>  = 自动编号保存当前点击')
    rospy.loginfo('    <名称>  = 以给定名称保存当前点击')
    rospy.loginfo('    list    = 列出所有已保存的点')
    rospy.loginfo('    undo    = 删除最后一个点')
    rospy.loginfo('    save    = 保存到 YAML 文件')
    rospy.loginfo('    quit    = 退出并自动保存')
    rospy.loginfo('=' * 60)
    rospy.loginfo('')

    try:
        while not rospy.is_shutdown():
            # 非阻塞读取 stdin
            if sys.stdin.isatty():
                line = sys.stdin.readline()
                cmd = line.strip()

                if not cmd:
                    continue  # 空行跳过

                if cmd.lower() == 'quit' or cmd.lower() == 'q':
                    if marker.points:
                        marker.save()
                    rospy.loginfo('[Marker] Bye.')
                    break
                elif cmd.lower() == 'save':
                    marker.save()
                elif cmd.lower() == 'list':
                    marker.list_points()
                elif cmd.lower() == 'undo':
                    marker.undo()
                elif cmd.lower().startswith('save '):
                    # 自定义文件名
                    marker.save(cmd[5:].strip())
                else:
                    # 当作点名称
                    marker.add_point(cmd)
            else:
                rospy.sleep(0.1)

    except KeyboardInterrupt:
        if marker.points:
            marker.save()
        rospy.loginfo('[Marker] Interrupted. Bye.')


if __name__ == '__main__':
    main()
