#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""豆包 ASR 节点：录音 → 豆包语音识别 → 检测开始口令 → 发布 /start。

豆包 ASR 使用火山引擎语音识别大模型极速版 HTTP API。
认证: X-Api-App-Key + X-Api-Access-Key。
录音: 使用 PulseAudio parec（支持蓝牙麦克风），不再依赖 pyaudio/ALSA。
"""

import rospy
import os
import sys
import time
import json
import tempfile
import wave
import requests
import base64
import uuid as _uuid
import subprocess
from std_msgs.msg import String

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', 'abot_vlm', 'scripts'))
from API_KEY_DOUBAO import SPEECH_APPID, SPEECH_TOKEN, SPEECH_ASR_RESOURCE_ID

# ---- ASR 配置 ----
ASR_API_URL = "https://openspeech.bytedance.com/api/v3/auc/bigmodel/recognize/flash"

# ---- 录音参数 ----
SAMPLE_RATE = 8000  # 蓝牙麦克风原生 8000Hz，16000 会被 PulseAudio 静音
CHANNELS = 1
RECORD_SECONDS = 3  # was 4
START_KEYWORDS = ('开始比赛', '比赛开始')


def _find_bluetooth_source():
    """查找 PulseAudio 蓝牙麦克风源名称。"""
    try:
        out = subprocess.check_output(['pactl', 'list', 'short', 'sources'], stderr=subprocess.STDOUT)
        for line in out.decode('utf-8').strip().split('\n'):
            if 'bluez_source' in line:
                name = line.split()[1]
                rospy.loginfo('[DoubaoASR] Found BT source: %s', name)
                return name
    except Exception:
        pass
    return None


def record_audio(filename, duration=RECORD_SECONDS):
    """PulseAudio parec 录音（支持蓝牙麦克风），保存为 WAV。"""
    bt_source = _find_bluetooth_source()
    cmd = ['parec', '--format=s16le', '--rate={}'.format(SAMPLE_RATE),
           '--channels={}'.format(CHANNELS), '--latency-msec=10']
    if bt_source:
        cmd += ['--device=' + bt_source]
    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    bytes_per_sec = SAMPLE_RATE * CHANNELS * 2  # 16-bit = 2 bytes
    total_bytes = bytes_per_sec * duration
    raw_data = b''
    try:
        while len(raw_data) < total_bytes:
            chunk = proc.stdout.read(min(4096, total_bytes - len(raw_data)))
            if not chunk:
                break
            raw_data += chunk
    finally:
        proc.terminate()
        proc.wait()
    with wave.open(filename, 'wb') as wf:
        wf.setnchannels(CHANNELS)
        wf.setsampwidth(2)
        wf.setframerate(SAMPLE_RATE)
        wf.writeframes(raw_data)


class DoubaoASR(object):
    """豆包语音识别，检测比赛开始指令。"""

    def __init__(self):
        self.start_pub = rospy.Publisher('/start', String, queue_size=10, latch=True)
        self.appid = SPEECH_APPID
        self.token = SPEECH_TOKEN
        self.resource_id = SPEECH_ASR_RESOURCE_ID
        rospy.loginfo('[DoubaoASR] Ready. resource_id=%s appid=%s',
                      self.resource_id, self.appid)

        # PulseAudio 录音源检查
        try:
            result = subprocess.check_output(['pactl', 'list', 'short', 'sources'], stderr=subprocess.STDOUT)
            sources = [l for l in result.decode('utf-8').strip().split('\n') if l.strip()]
            rospy.loginfo('[DoubaoASR] PulseAudio sources: %d OK', len(sources))
        except Exception as e:
            rospy.logwarn('[DoubaoASR] Cannot enumerate PulseAudio sources: %s', e)

    def run(self):
        # 通过豆包 TTS 播提示音（TTS 在 step4 已启动，此时必然就绪）
        tts_pub = rospy.Publisher('/voiceWords', String, queue_size=1)
        rospy.sleep(0.3)  # 等 TTS 订阅连接
        tts_pub.publish(String(data='请说开始比赛'))
        rospy.sleep(1.5)  # 等 TTS 播完
        rate = rospy.Rate(0.5)
        listen_timeout = float(rospy.get_param('~listen_timeout_s', 120.0))
        deadline = time.time() + listen_timeout
        rospy.loginfo('[DoubaoASR] Listening for keywords: %s (timeout %.1fs)',
                      ', '.join(START_KEYWORDS), listen_timeout)
        while not rospy.is_shutdown() and time.time() < deadline:
            if rospy.get_param('/start', False):
                rospy.loginfo('[DoubaoASR] /start already set, exiting')
                break
            try:
                with tempfile.NamedTemporaryFile(suffix='.wav', delete=False) as f:
                    tmp_path = f.name
                record_audio(tmp_path, duration=RECORD_SECONDS)
                result = self._recognize(tmp_path)
                os.unlink(tmp_path)

                if result and any(keyword in result for keyword in START_KEYWORDS):
                    rospy.loginfo('[DoubaoASR] Start keyword detected: %s -> /start', result)
                    self.start_pub.publish(String(data='True'))
                    rospy.set_param('/start', True)
                    rospy.sleep(0.5)
                    break
                elif result:
                    rospy.loginfo('[DoubaoASR] Heard: %s', result[:60])
            except Exception as e:
                rospy.logwarn('[DoubaoASR] %s', e)
            rate.sleep()
        else:
            rospy.loginfo('[DoubaoASR] Listen window expired, exiting')

    def _recognize(self, audio_path):
        """调用豆包语音识别大模型极速版 HTTP API。"""
        if not self.token:
            rospy.logerr('[DoubaoASR] API key not set')
            return None
        try:
            with open(audio_path, 'rb') as f:
                audio_data = f.read()
            headers = {
                'X-Api-App-Key': self.appid,
                'X-Api-Access-Key': self.token,
                'X-Api-Resource-Id': self.resource_id,
                'X-Api-Request-Id': str(_uuid.uuid4()),
                'X-Api-Sequence': '-1',
                'Content-Type': 'application/json',
            }
            body = {
                'user': {'uid': 'abot_robot'},
                'audio': {
                    'format': 'wav',
                    'data': base64.b64encode(audio_data).decode('utf-8'),
                },
                'request': {
                    'model_name': 'bigmodel',
                    'enable_itn': True,
                    'enable_punc': True,
                },
            }
            resp = requests.post(ASR_API_URL, headers=headers, json=body, timeout=10)
            status_code = resp.headers.get('X-Api-Status-Code', '')
            message = resp.headers.get('X-Api-Message', '')
            logid = resp.headers.get('X-Tt-Logid', '')
            if status_code != '20000000':
                if status_code == '20000003':  # no valid speech — 静默监听期间正常
                    rospy.logdebug('[DoubaoASR] Silence (no valid speech)')
                else:
                    rospy.logwarn('[DoubaoASR] ASR failed: http=%d code=%s msg=%s logid=%s',
                                  resp.status_code, status_code, message, logid)
                return None

            data = resp.json()
            result = data.get('result', {})
            if isinstance(result, dict):
                return result.get('text', '').strip()
            if isinstance(result, list) and result:
                return result[0].get('text', '').strip()
            return data.get('text', '').strip()
        except Exception as e:
            rospy.logerr('[DoubaoASR] Recognition failed: %s', e)
            return None


if __name__ == '__main__':
    rospy.init_node('doubao_asr')
    DoubaoASR().run()
