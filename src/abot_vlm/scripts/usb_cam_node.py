#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""USB 相机驱动节点：读取本地摄像头，发布 sensor_msgs/Image 到 /usb_cam/image_raw。

为 doubao.py(豆包 VLM) 提供图像源。doubao.py 自带 imgmsg_to_cv2 且按 bgr8 解析，
因此本节点直接发布 bgr8 编码、手动打包 Image 消息，不依赖 cv_bridge（与 doubao.py 风格一致）。

可调参数(均为 ~private, 由 launch 覆盖):
  ~video_device   摄像头设备 (默认 0, 对应 /dev/video0; 也可填 "/dev/video0")
  ~image_width    采集宽度 (默认 640, 对齐 perception.yaml)
  ~image_height   采集高度 (默认 480)
  ~fps            发布帧率 (默认 15)
  ~frame_id       图像坐标系 (默认 usb_cam, 对齐 robot.yaml)
  ~topic          发布话题 (默认 /usb_cam/image_raw, 对齐 doubao.py 订阅)
"""
import sys
reload(sys)
sys.setdefaultencoding('utf-8')
import rospy
import cv2
from sensor_msgs.msg import Image


def cv2_to_imgmsg(cv_image, frame_id):
    """手动将 OpenCV BGR 图像打包为 sensor_msgs/Image (bgr8), 不依赖 cv_bridge。

    与 doubao.py 的 imgmsg_to_cv2 对应: 发 bgr8, 对端无需 cvtColor 直接使用。
    """
    msg = Image()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = frame_id
    msg.height = cv_image.shape[0]
    msg.width = cv_image.shape[1]
    msg.encoding = 'bgr8'
    msg.is_bigendian = 0
    msg.step = cv_image.shape[1] * 3  # width * channels(3) * bytes(1)
    msg.data = cv_image.tobytes()
    return msg


def _open_capture(device, width, height):
    """打开摄像头并设置分辨率。device 可为 int(0) 或字符串("/dev/video0")。"""
    # 数字字符串("0") 转 int; "/dev/videoX" 保持字符串
    if isinstance(device, str) and device.isdigit():
        device = int(device)
    cap = cv2.VideoCapture(device)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    return cap


def main():
    rospy.init_node('usb_cam_node', anonymous=False)

    device = rospy.get_param('~video_device', 0)
    width = rospy.get_param('~image_width', 640)
    height = rospy.get_param('~image_height', 480)
    fps = rospy.get_param('~fps', 15)
    frame_id = rospy.get_param('~frame_id', 'usb_cam')
    topic = rospy.get_param('~topic', '/usb_cam/image_raw')

    pub = rospy.Publisher(topic, Image, queue_size=2)

    cap = _open_capture(device, width, height)
    if not cap.isOpened():
        rospy.logerr('[usb_cam] 无法打开摄像头 device=%s, 检查 /dev/video* 与权限', str(device))
        return

    # 暖机：丢弃前几帧（USB 摄像头刚打开时帧可能全黑/曝光未稳定）
    warmup_frames = 5
    warmup_mean = 0.0
    for i in range(warmup_frames):
        ret, _ = cap.read()
        if ret:
            warmup_mean += 1.0
    warmup_ok = warmup_mean >= (warmup_frames - 1)  # 至少 N-1 帧成功
    if not warmup_ok:
        rospy.logwarn('[usb_cam] 暖机期间仅成功 %d/%d 帧，图像可能异常', int(warmup_mean), warmup_frames)

    actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    rospy.loginfo('[usb_cam] 已打开 device=%s, 分辨率 %dx%d, 发布 %s @ %dHz',
                  str(device), actual_w, actual_h, topic, fps)

    if actual_w != width or actual_h != height:
        rospy.logwarn('[usb_cam] 请求 %dx%d, 实际 %dx%d — 摄像头不支持请求分辨率!',
                      width, height, actual_w, actual_h)

    rate = rospy.Rate(fps)
    fail_count = 0
    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if not ret or frame is None:
            fail_count += 1
            # 偶发丢帧只记一次, 连续失败才升级为 error 并尝试重开
            if fail_count == 1:
                rospy.logwarn('[usb_cam] 读取帧失败, 重试中...')
            if fail_count >= 30:
                rospy.logerr('[usb_cam] 连续 30 帧读取失败, 尝试重新打开摄像头')
                cap.release()
                cap = _open_capture(device, width, height)
                fail_count = 0
            rate.sleep()
            continue
        fail_count = 0

        msg = cv2_to_imgmsg(frame, frame_id)
        pub.publish(msg)
        rate.sleep()

    cap.release()
    rospy.loginfo('[usb_cam] 节点退出, 摄像头已释放')


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
