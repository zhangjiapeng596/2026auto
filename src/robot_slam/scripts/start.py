#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
语音唤醒节点：播放提示音 → 等待 5s 录音 → 发布 /start 信号
替代 Snowboy 热词检测，在无麦克风或 SSH 模式下也可工作。
"""
import rospy
import os
import sys
import subprocess
import signal
import threading
from std_msgs.msg import String

# Python 2.7 兼容
reload(sys)
sys.setdefaultencoding('utf-8')
DEVNULL = open(os.devnull, 'w')

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
MP3_PATH = os.path.join(SCRIPT_DIR, 'start_record.mp3')

# HDMI 扬声器（与 doubao_tts.py 一致）
PULSE_SINK = 'alsa_output.pci-0000_00_0e.0.hdmi-stereo-extra1'

interrupted = False

def signal_handler(sig, frame):
    global interrupted
    interrupted = True

def play_mp3(path):
    """通过 HDMI 扬声器播放 MP3 提示音（Python 2.7 兼容）。"""
    env = os.environ.copy()
    env['PULSE_SINK'] = PULSE_SINK
    env['DISPLAY'] = ':0'
    env['XAUTHORITY'] = '/run/user/1000/gdm/Xauthority'

    for cmd in (
        ['ffplay', '-nodisp', '-autoexit', path],
        ['mplayer', '-really-quiet', '-nolirc', path],
    ):
        try:
            rospy.loginfo('[Wakeup] Trying: %s', ' '.join(cmd))
            proc = subprocess.Popen(cmd, env=env,
                                    stdout=DEVNULL, stderr=DEVNULL)
            t = threading.Timer(15.0, proc.kill)
            t.start()
            ret = proc.wait()
            t.cancel()
            if ret == 0:
                return True
            rospy.logwarn('[Wakeup] %s exited with %d', cmd[0], ret)
        except Exception as e:
            rospy.logwarn('[Wakeup] Player %s failed: %s', cmd[0], str(e))
    return False

if __name__ == '__main__':
    rospy.init_node('game_node', anonymous=True)
    signal.signal(signal.SIGINT, signal_handler)

    pub = rospy.Publisher('/start', String, queue_size=1)
    rospy.sleep(3)  # 等 ROS/PulseAudio 初始化

    if os.path.isfile(MP3_PATH):
        rospy.loginfo('[Wakeup] Playing: %s', MP3_PATH)
        ok = play_mp3(MP3_PATH)
        rospy.loginfo('[Wakeup] Playback %s', 'done' if ok else 'FAILED')
    else:
        rospy.logwarn('[Wakeup] MP3 not found: %s', MP3_PATH)

    # 5 秒倒计时（可打断）
    rospy.loginfo('[Wakeup] Recording window (5s)...')
    for i in range(5, 0, -1):
        if interrupted:
            rospy.loginfo('[Wakeup] Interrupted')
            break
        rospy.sleep(1)

    if not interrupted:
        msg = String()
        msg.data = 'True'
        pub.publish(msg)
        rospy.set_param('/start', True)
        rospy.loginfo('[Wakeup] /start published (True)')

    rospy.spin()
