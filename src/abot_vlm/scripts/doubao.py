#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from volcenginesdkarkruntime import Ark
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image as ROSImage
from std_msgs.msg import String
import os
import sys
import json
import re
import hashlib
from PIL import Image, ImageFont, ImageDraw
import time
import base64

# API Key 来源优先级: 环境变量 DOUBAO_KEY > API_KEY_DOUBAO.py(被 .gitignore, 现场自备)
# 不在源码中硬编码密钥 (遵循需求文档 7.10 不提交密钥)。
DOUBAO_KEY = os.environ.get('DOUBAO_KEY')
if not DOUBAO_KEY:
    try:
        from API_KEY_DOUBAO import DOUBAO_KEY
    except Exception:
        DOUBAO_KEY = None


def _get_image_path():
    """图片保存/读取路径，可配置，默认放系统临时目录，避免硬编码 /home/abot。"""
    default_dir = os.environ.get('ABOT_VLM_TMP',
                                 os.path.join(os.path.expanduser('~'), '.ros', 'abot_vlm'))
    tmp_dir = rospy.get_param('~temp_dir', default_dir) if rospy.is_initialized() else default_dir
    try:
        if not os.path.isdir(tmp_dir):
            os.makedirs(tmp_dir)
    except OSError:
        tmp_dir = '/tmp'
    return os.path.join(tmp_dir, 'vl_now.jpg')


client = Ark(
    base_url="https://ark.cn-beijing.volces.com/api/v3",
    api_key=DOUBAO_KEY
)

def imgmsg_to_cv2(img_msg):
    dtype = np.dtype("uint8")  # Hardcode to 8 bits...
    dtype = dtype.newbyteorder('>' if img_msg.is_bigendian else '<')
    image_opencv = np.ndarray(shape=(img_msg.height, img_msg.width, 3), dtype=dtype, buffer=img_msg.data)

    # If the byte order is different between the message and the system.
    if img_msg.is_bigendian == (sys.byteorder == 'little'):
        image_opencv = image_opencv.byteswap().newbyteorder()

    # Convert to BGR if the encoding is not already BGR
    if img_msg.encoding == "rgb8":
        image_opencv = cv2.cvtColor(image_opencv, cv2.COLOR_RGB2BGR)
    elif img_msg.encoding == "mono8":
        image_opencv = cv2.cvtColor(image_opencv, cv2.COLOR_GRAY2BGR)
    elif img_msg.encoding != "bgr8":
        rospy.logerr("Unsupported encoding: %s", img_msg.encoding)
        return None

    return image_opencv

def doubao_vision_api(PROMPT=None, img_path=None):
    '''
    豆包大模型视觉语言多模态功能
    '''
    if img_path is None:
        img_path = _get_image_path()
    if PROMPT is None:
        PROMPT = rospy.get_param('/perception/prompt_template',
            '你是一个机器人比赛的视觉识别系统。'
            '请识别图中围栏上的任务信息图像，'
            '输出图像内容和对应的任务点编号（31-33, 40-42, 49-51 之一）。'
            '回复格式必须是 JSON：'
            '{"target_cell": <数字>, "content": "<图像内容描述>", "confidence": <0-1之间的浮点数>}'
            '如果无法识别，设置 confidence 为 0。')
    # 编码为base64数据
    with open(img_path, 'rb') as image_file:
        image = 'data:image/jpeg;base64,' + base64.b64encode(image_file.read()).decode('utf-8')
    
    # 向豆包大模型发起请求
    response = client.chat.completions.create(
        model="doubao-1-5-vision-pro-32k-250115",
        messages=[
            {
                "role": "user",
                "content": [
                    {"type": "text", "text": PROMPT},
                    {
                        "type": "image_url",
                        "image_url": {
                            "url": image
                        }
                    },
                ]
            }
        ],
    )
    
    # 解析大模型返回结果
    result_str = response.choices[0].message.content.strip()
    result = str(result_str)
        
    print('豆包大模型调用成功！')
    print('结果:', result)

    return result


def parse_vlm_result(raw_text, image_id):
    """把 VLM 原始文本稳健解析为状态机约定的 JSON 结构。

    状态机 _on_vision_result 期望: {target_cell, content, confidence, image_id, timestamp}
    (与 mock_vlm.py 契约一致)。模型可能输出裸 JSON、带 markdown ```json 包裹、或夹杂文字，
    都尝试提取; 失败时返回 confidence=0, 让状态机走正常重试而非静默卡死。
    """
    target_cell, content, confidence = None, str(raw_text), 0.0
    try:
        # 从文本中抓第一个 {...} JSON 片段 (兼容 ```json 包裹和多余文字)
        m = re.search(r'\{.*\}', raw_text, re.DOTALL)
        if m:
            data = json.loads(m.group(0))
            target_cell = data.get('target_cell')
            content = data.get('content', content)
            confidence = float(data.get('confidence', 0.0))
            if target_cell is not None:
                target_cell = int(target_cell)
    except (ValueError, TypeError) as e:
        rospy.logwarn('[VLM] 结果解析失败, 按 confidence=0 处理: %s', str(e))
        target_cell, confidence = None, 0.0

    return {
        'target_cell': target_cell,
        'content': content,
        'confidence': confidence,
        'image_id': image_id,
        'timestamp': rospy.Time.now().to_sec(),
    }


# 模块级 publisher, 在 main() 中初始化一次, 避免每次回调重建丢消息
_result_pub = None


def top_view_shot(image_msg):
    '''
    这里接收来自话题/usb_cam/image_raw的ROS图像格式的消息，并保存图像，是否拍照用的参数服务器，然后设置参数就行，注意要加命名空间路径
    '''
    # 从参数服务器获取 im_flag (上升沿触发拍照)
    im_flag = rospy.get_param('/top_view_shot_node/im_flag', 255)
    if im_flag != 1:
        return

    img_path = _get_image_path()
    # 先复位 flag, 防止本次 API 阻塞期间重复触发
    rospy.set_param('/top_view_shot_node/im_flag', 255)

    # 用图像字节哈希 + 时间戳生成唯一 image_id, 供状态机去重 (seen_image_ids)
    img_hash = hashlib.md5(bytes(image_msg.data)).hexdigest()[:8]
    image_id = 'vl_%s_%d' % (img_hash, int(rospy.Time.now().to_sec()))

    result = None
    try:
        img_bgr = imgmsg_to_cv2(image_msg)
        if img_bgr is None:
            raise ValueError('图像转换失败 (不支持的编码)')
        rospy.loginfo('[VLM] 保存图像至 %s', img_path)
        cv2.imwrite(img_path, img_bgr)
        cv2.waitKey(1)

        # 调用豆包视觉大模型API
        raw = doubao_vision_api(img_path=img_path)
        rospy.loginfo('[VLM] 原始结果: %s', raw)
        result = parse_vlm_result(raw, image_id)
    except Exception as e:
        # API/网络/编码任意失败都发 confidence=0, 让状态机重试而非超时卡死
        rospy.logerr('[VLM] 识别失败: %s', str(e))
        result = {
            'target_cell': None, 'content': 'error: %s' % str(e),
            'confidence': 0.0, 'image_id': image_id,
            'timestamp': rospy.Time.now().to_sec(),
        }

    # 发布约定 JSON 结构
    payload = json.dumps(result, ensure_ascii=False)
    rospy.loginfo('[VLM] 发布 vision_result: %s', payload)
    if _result_pub is not None:
        _result_pub.publish(String(data=payload))
    else:
        rospy.logwarn('[VLM] publisher 未初始化, 丢弃结果')

def main():
    global _result_pub
    rospy.init_node('top_view_shot_node', anonymous=True)
    if DOUBAO_KEY is None:
        rospy.logwarn('[VLM] 未找到 DOUBAO_KEY (环境变量或 API_KEY_DOUBAO.py), API 调用将失败')
    # 全局话题 /vision_result, 与状态机订阅一致
    _result_pub = rospy.Publisher('/vision_result', String, queue_size=10)
    # 初始化触发参数为 255(非触发态)
    rospy.set_param('/top_view_shot_node/im_flag', 255)
    rospy.Subscriber('/usb_cam/image_raw', ROSImage, top_view_shot)
    rospy.loginfo('视觉大模型模块导入成功！')
    rospy.loginfo('准备识别...')

    rospy.spin()

if __name__ == '__main__':
    main()

