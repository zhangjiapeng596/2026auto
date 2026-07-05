#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import String
def voice_publisher():
    # 初始化 ROS 节点
    rospy.init_node('voice_publisher', anonymous=True)
    # 创建一个发布者，发布类型为 String 的消息到主题 '/voiceWords'
    tts_pub = rospy.Publisher('/voiceWords', String, queue_size=10)
    # 设置播报频率为每秒1次
    rate = rospy.Rate(1)
    # 待发布的消息内容
    tts_str = "请输入想要语音播报的内容"
    # 在节点没有关闭的情况下循环执行
    while not rospy.is_shutdown():
        # 打印消息到日志
        rospy.loginfo(tts_str)
        # 发布消息到 '/voiceWords' 主题
        tts_pub.publish(tts_str)  # 注意：应将字符串封装成 String 消息类型
        # 控制发布频率
        rate.sleep()
if __name__ == '__main__':
    try:
        # 执行主程序
        voice_publisher()
	# 用户按下 Ctrl + C 终止节点时会触发 ROSInterruptException 异常，这里捕获异常并忽略，使程序能够优雅地退出
    except rospy.ROSInterruptException:
        pass

