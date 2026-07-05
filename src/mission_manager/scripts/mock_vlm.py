#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""仿真用 Mock VLM 节点。读取 competition_field.yaml 中 vision_to_task 映射，
   按 vision_positions 顺序模拟图像识别，返回对应任务区号。

用法:
  rosrun mission_manager mock_vlm.py
  rosrun mission_manager mock_vlm.py _confidence:=0.9

订阅: /top_view_shot_node/im_flag (ROS param, 上升沿触发)
发布: /vision_result (std_msgs/String JSON)
"""
import os
import sys
reload(sys)
sys.setdefaultencoding('utf-8')
import rospy
import json
import yaml
from std_msgs.msg import String

# config 搜索路径
CONFIG_PATHS = [
    os.path.join(os.path.dirname(__file__), '..', '..', '..', 'config'),
    os.path.join(os.path.expanduser('~'), '3X2KY2', 'config'),
    os.path.join(os.environ.get('WS_PATH', ''), 'config'),
]


def _find_config(filename):
    for d in CONFIG_PATHS:
        p = os.path.join(os.path.abspath(d), filename)
        if os.path.isfile(p):
            return p
    raise IOError('Config not found: %s' % filename)


class MockVLM(object):
    """模拟 VLM：按 vision_positions 顺序依次返回 vision_to_task 映射的任务区。"""

    def __init__(self):
        # 读取场地配置
        config_path = _find_config('competition_field.yaml')
        with open(config_path, 'r') as f:
            cfg = yaml.safe_load(f)

        self.vision_positions = cfg.get('vision_positions', [5, 37, 45, 77])
        self.vision_to_task = cfg.get('vision_to_task',
                                       {5: 41, 37: 33, 45: 50, 77: 31})
        self.confidence = rospy.get_param('~confidence', 0.85)
        self.trigger_param = rospy.get_param('~trigger_param',
                                             '/top_view_shot_node/im_flag')

        self.current_index = 0
        self.result_pub = rospy.Publisher('/vision_result', String, queue_size=10)

        rospy.loginfo('[MockVLM] vision_positions: %s', self.vision_positions)
        rospy.loginfo('[MockVLM] vision_to_task: %s', self.vision_to_task)
        rospy.loginfo('[MockVLM] confidence: %.2f', self.confidence)

    def run(self):
        rate = rospy.Rate(5)
        last_flag = 0
        while not rospy.is_shutdown():
            current_flag = rospy.get_param(self.trigger_param, 0)
            if current_flag == 1 and last_flag == 0:
                self._do_recognition()
            last_flag = current_flag
            rate.sleep()

    def _do_recognition(self):
        if self.current_index >= len(self.vision_positions):
            rospy.logwarn('[MockVLM] All %d tasks done', len(self.vision_positions))
            self._publish_result(None, 0, 'no_more_tasks')
            return

        vision_cell = self.vision_positions[self.current_index]
        vinfo = self.vision_to_task.get(vision_cell, {})
        # 兼容两种格式: {target, yaw_rad} 或纯数字
        if isinstance(vinfo, dict):
            target_cell = vinfo.get('target')
        else:
            target_cell = vinfo
        image_id = 'mock_image_%d' % (self.current_index + 1)

        rospy.loginfo('[MockVLM] Task %d/%d: vision=%d → target=%d (conf=%.2f)',
                      self.current_index + 1, len(self.vision_positions),
                      vision_cell, target_cell, self.confidence)

        self._publish_result(target_cell, self.confidence, image_id)
        self.current_index += 1
        rospy.set_param(self.trigger_param, 0)

    def _publish_result(self, target_cell, confidence, image_id):
        result = {
            'target_cell': target_cell,
            'content': image_id,
            'confidence': confidence,
            'image_id': image_id,
            'timestamp': rospy.Time.now().to_sec(),
        }
        msg = String(data=json.dumps(result))
        self.result_pub.publish(msg)
        rospy.loginfo('[MockVLM] Published: %s', json.dumps(result, ensure_ascii=False))


if __name__ == '__main__':
    rospy.init_node('mock_vlm')
    MockVLM().run()
