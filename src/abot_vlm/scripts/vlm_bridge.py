#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""VLM Bridge — Python 2.7 ROS 节点，通过 subprocess 调用 doubao_worker.py (py3.9)。

替代原 doubao.py：将 ROS 通信（py2.7）与豆包 API（py3.9）解耦。
Bridge 负责: 订阅图像 → 保存文件 → 调用 worker → 解析结果 → 发布 /vision_result
Worker 负责: 纯豆包 API 调用，零 ROS 依赖
"""
import rospy
import cv2
import numpy as np
import os, sys, json, re, hashlib, subprocess, threading, time
from sensor_msgs.msg import Image as ROSImage
from std_msgs.msg import String

reload(sys)
sys.setdefaultencoding('utf-8')

# Worker 路径: 与 bridge 同目录
_WORKER = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'doubao_worker.py')
# py3.9 解释器
_PY39 = '/home/abot/anaconda3/envs/py39/bin/python3.9'


def _get_image_path():
    default_dir = os.environ.get('ABOT_VLM_TMP',
                                 os.path.join(os.path.expanduser('~'), '.ros', 'abot_vlm'))
    tmp_dir = rospy.get_param('~temp_dir', default_dir)
    try:
        if not os.path.isdir(tmp_dir):
            os.makedirs(tmp_dir)
    except OSError:
        tmp_dir = '/tmp'
    return os.path.join(tmp_dir, 'vl_now.jpg')


def imgmsg_to_cv2(img_msg):
    dtype = np.dtype("uint8")
    dtype = dtype.newbyteorder('>' if img_msg.is_bigendian else '<')
    image_opencv = np.ndarray(shape=(img_msg.height, img_msg.width, 3),
                              dtype=dtype, buffer=img_msg.data)
    if img_msg.is_bigendian == (sys.byteorder == 'little'):
        image_opencv = image_opencv.byteswap().newbyteorder()
    if img_msg.encoding == "rgb8":
        image_opencv = cv2.cvtColor(image_opencv, cv2.COLOR_RGB2BGR)
    elif img_msg.encoding == "mono8":
        image_opencv = cv2.cvtColor(image_opencv, cv2.COLOR_GRAY2BGR)
    elif img_msg.encoding != "bgr8":
        rospy.logerr("Unsupported encoding: %s", img_msg.encoding)
        return None
    return image_opencv


def call_worker(img_path, prompt, image_id, timeout_s=30):
    """调用 py3.9 worker 进程，返回解析后的 JSON dict。"""
    cmd = [
        _PY39, _WORKER,
        '--image', img_path,
        '--image-id', image_id,
    ]
    if prompt:
        cmd += ['--prompt', prompt.encode('utf-8') if isinstance(prompt, unicode) else prompt]

    try:
        proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        # Python 2.7: communicate() 无 timeout 参数, 用 Timer 实现超时
        t = threading.Timer(timeout_s, proc.kill)
        try:
            t.start()
            stdout, stderr = proc.communicate()
        finally:
            t.cancel()
        if proc.returncode != 0:
            if proc.returncode == -9:
                rospy.logerr('[VLM] Worker timeout (%ds)', timeout_s)
            else:
                rospy.logerr('[VLM] Worker failed (rc=%d): %s', proc.returncode, stderr.strip())
        result = json.loads(stdout.strip()) if stdout.strip() else {}
    except Exception as e:
        rospy.logerr('[VLM] Worker exception: %s', str(e))
        result = {}

    # 确保返回结构完整
    result.setdefault('target_cell', None)
    result.setdefault('content', 'worker error')
    result.setdefault('confidence', 0.0)
    result.setdefault('image_id', image_id)
    result.setdefault('timestamp', rospy.Time.now().to_sec())
    return result


class VlmBridge(object):
    def __init__(self):
        self.result_pub = rospy.Publisher('/vision_result', String, queue_size=10)
        rospy.set_param('/top_view_shot_node/im_flag', 255)
        self.image_sub = rospy.Subscriber('/usb_cam/image_raw', ROSImage, self._on_image)
        self.prompt = rospy.get_param('/perception/prompt_template', '')
        rospy.loginfo('[VLM Bridge] Ready. Worker: %s', _WORKER)

    def _on_image(self, image_msg):
        im_flag = rospy.get_param('/top_view_shot_node/im_flag', 255)
        if im_flag != 1:
            return
        # 复位 flag 防重复触发
        rospy.set_param('/top_view_shot_node/im_flag', 255)

        img_path = _get_image_path()
        img_id = 'vl_%s_%d' % (hashlib.md5(bytes(image_msg.data)).hexdigest()[:8],
                               int(rospy.Time.now().to_sec()))

        # 保存图像
        img_bgr = imgmsg_to_cv2(image_msg)
        if img_bgr is None:
            rospy.logerr('[VLM] 图像转换失败')
            self._publish_error('image conversion failed', img_id)
            return

        cv2.imwrite(img_path, img_bgr)
        rospy.loginfo('[VLM] Image saved: %s', img_path)

        # 在 ABOT 屏幕显示当前捕获图像
        try:
            cv2.imshow('VLM Capture', img_bgr)
            cv2.waitKey(1)
        except Exception:
            pass

        # 亮度检查：过暗/过曝时 warn（帮助诊断现场问题，不阻断流程）
        mean_brightness = img_bgr.mean()
        if mean_brightness < 10.0:
            rospy.logwarn('[VLM] Image very dark (mean=%.1f), possible camera/lens cap issue', mean_brightness)
        elif mean_brightness > 245.0:
            rospy.logwarn('[VLM] Image overexposed (mean=%.1f), possible glare', mean_brightness)

        # 保存调试图像（带时间戳，不覆盖，便于赛后回溯）
        debug_dir = os.path.join(os.path.dirname(img_path), 'debug')
        try:
            if not os.path.isdir(debug_dir):
                os.makedirs(debug_dir)
            debug_img = os.path.join(debug_dir, '{}.jpg'.format(img_id))
            cv2.imwrite(debug_img, img_bgr)
            rospy.loginfo('[VLM] Debug image: %s', debug_img)
        except Exception:
            pass

        # 调用 worker (py3.9)
        rospy.loginfo('[VLM] Calling worker...')
        result = call_worker(img_path, self.prompt, img_id)

        # 保存调试响应
        try:
            debug_resp = os.path.join(debug_dir, '{}_response.json'.format(img_id))
            with open(debug_resp, 'w') as f:
                json.dump(result, f, ensure_ascii=False, indent=2)
        except Exception:
            pass

        # 发布结果
        payload = json.dumps(result, ensure_ascii=False)
        rospy.loginfo('[VLM] Result: %s', payload)
        self.result_pub.publish(String(data=payload))

    def _publish_error(self, msg, img_id):
        err = {'target_cell': None, 'content': 'error: %s' % msg,
               'confidence': 0.0, 'image_id': img_id,
               'timestamp': rospy.Time.now().to_sec()}
        self.result_pub.publish(String(data=json.dumps(err, ensure_ascii=False)))


if __name__ == '__main__':
    rospy.init_node('top_view_shot_node', anonymous=True)
    VlmBridge()
    rospy.spin()
