#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Mock TTS 仿真节点：订阅 /voiceWords，模拟播报后发布 /tts_done。"""
import sys
reload(sys)
sys.setdefaultencoding('utf-8')
import rospy
import re
from std_msgs.msg import String


class MockTTS(object):
    """模拟 TTS 节点：收到语音文本后等待，发布完成信号。"""

    def __init__(self):
        self.speech_rate = rospy.get_param('~speech_rate', 3.0)  # 汉字/秒
        self.min_duration = rospy.get_param('~min_duration', 0.5)  # 最短等待
        self.max_duration = rospy.get_param('~max_duration', 8.0)  # 最长等待

        self.tts_done_pub = rospy.Publisher('/tts_done', String, queue_size=10)
        rospy.Subscriber('/voiceWords', String, self._on_voice)

        rospy.loginfo('[MockTTS] Ready. speech_rate=%.1f char/s, duration=[%.1f, %.1f]s',
                      self.speech_rate, self.min_duration, self.max_duration)

    def _count_chinese(self, text):
        """估算中文文本的朗读时长。"""
        if not isinstance(text, unicode):
            text_u = text.decode('utf-8')
        else:
            text_u = text
        # 统计中文字符数（Unicode 范围）
        chinese = len(re.findall(ur'[一-鿿]', text_u))
        # 统计英文/数字字符数（逐个字符，非词组）
        english = len(re.findall(ur'[a-zA-Z0-9]', text_u))
        return chinese + english * 0.5  # 英文字符按半个汉字算

    def _on_voice(self, msg):
        """收到播报请求，模拟播放后发布完成信号。"""
        text = msg.data
        rospy.loginfo('[MockTTS] Speaking: "%s"', text)

        # 估算时长（speech_rate 零值保护）
        char_count = self._count_chinese(text)
        rate = max(self.speech_rate, 0.1)
        duration = max(self.min_duration, min(self.max_duration, char_count / rate))
        rospy.loginfo('[MockTTS] Estimated %.1f chars, playing for %.2fs',
                      char_count, duration)

        # 模拟播放（非阻塞 sleep）
        rospy.sleep(duration)

        # 发布完成信号
        done_msg = String()
        done_msg.data = text
        self.tts_done_pub.publish(done_msg)
        rospy.loginfo('[MockTTS] Done: "%s"', text)


if __name__ == '__main__':
    rospy.init_node('mock_tts')
    MockTTS()
    rospy.spin()
