#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""豆包 TTS 节点：订阅 /voiceWords，调用豆包语音合成 V1 HTTP API，mplayer 播放。

认证: Bearer;token (分号分隔，无空格)。
API: https://openspeech.bytedance.com/api/v1/tts (V1 非流式)
输出 /tts_done 通知播报完成。

Python 2.7 兼容: 使用 urllib2 代替 requests，避免 py2/py3 冲突。
"""

import rospy
import os
import sys
import subprocess
import base64
import tempfile
import urllib2
import json as _json
import uuid as _uuid
from std_msgs.msg import String

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', 'abot_vlm', 'scripts'))
from API_KEY_DOUBAO import SPEECH_APPID, SPEECH_TOKEN

# ---- TTS 配置 ----
TTS_API_URL = "https://openspeech.bytedance.com/api/v1/tts"
# 已确认可用的音色
TTS_VOICE_TYPE = "BV002_streaming"

# ---- 音频输出配置 ----
# 强制 TTS 使用 HDMI 扬声器输出，防止蓝牙耳机连接时劫持默认音频 sink
# PulseAudio sink 名，可通过 ROS param ~speaker_sink 覆盖
DEFAULT_SPEAKER_SINK = "alsa_output.pci-0000_00_0e.0.hdmi-stereo-extra1"


class DoubaoTTS(object):
    """豆包语音合成 + mplayer 播放。"""

    def __init__(self):
        self.tts_done_pub = rospy.Publisher('/tts_done', String, queue_size=10)
        rospy.Subscriber('/voiceWords', String, self._on_voice)
        self.appid = SPEECH_APPID
        self.token = SPEECH_TOKEN

        # 扬声器 sink：强制 TTS 输出到 HDMI/扬声器，不被蓝牙耳机劫持
        self.speaker_sink = rospy.get_param('~speaker_sink', DEFAULT_SPEAKER_SINK)
        self._check_sink()
        rospy.loginfo('[DoubaoTTS] Ready. voice=%s sink=%s', TTS_VOICE_TYPE, self.speaker_sink)

    def _check_sink(self):
        """验证 PulseAudio sink 存在，不存在时记录错误但不阻塞启动。

        Python 2.7 兼容: 使用 Popen 代替 check_output (后者不含 stderr kwarg)。
        """
        try:
            with open(os.devnull, 'w') as devnull:
                p = subprocess.Popen(
                    ['pactl', 'list', 'short', 'sinks'],
                    stdout=subprocess.PIPE, stderr=devnull)
                out, _ = p.communicate()
            if p.returncode != 0:
                rospy.logwarn('[DoubaoTTS] pactl 退出码=%d，无法验证 sink: %s', p.returncode, self.speaker_sink)
                return
            if self.speaker_sink not in out:
                available = [l.split()[1] for l in out.strip().split('\n') if l.strip()]
                rospy.logerr('[DoubaoTTS] 扬声器 sink 不存在: %s', self.speaker_sink)
                rospy.logerr('[DoubaoTTS] 可用的 sinks: %s', available)
                rospy.logerr('[DoubaoTTS] 将回退到 PulseAudio 默认设备 — 蓝牙耳机连接时 TTS 可能从耳机输出')
        except OSError as e:
            rospy.logwarn('[DoubaoTTS] pactl 不可用，无法验证 sink: %s', e)

    def _play_audio(self, args):
        """通过 subprocess 运行音频命令，强制 PULSE_SINK 指向扬声器。

        使用 subprocess (非 os.system) 避免 shell 注入风险。
        参数 args 为列表，不从 shell 解析。
        """
        env = os.environ.copy()
        env['PULSE_SINK'] = self.speaker_sink
        try:
            with open(os.devnull, 'w') as devnull:
                rc = subprocess.call(args, env=env, stdout=devnull, stderr=devnull)
            if rc != 0:
                rospy.logwarn('[DoubaoTTS] %s 退出码=%d (sink=%s)', args[0], rc, self.speaker_sink)
        except OSError as e:
            rospy.logerr('[DoubaoTTS] 无法运行 %s: %s', args[0], e)

    def _on_voice(self, msg):
        text = msg.data.strip()
        if not text:
            return
        rospy.loginfo('[DoubaoTTS] Speaking: %s', text[:50])

        if self.token:
            try:
                self._speak(text)
            except Exception as e:
                rospy.logerr('[DoubaoTTS] TTS failed: %s', e)
        else:
            # espeak 降级方案：同样强制输出到扬声器
            self._play_audio(['espeak', '-v', 'zh', text])

        self.tts_done_pub.publish(String(data=text))
        rospy.loginfo('[DoubaoTTS] Done: %s', text[:30])

    def _speak(self, text):
        """调用豆包 TTS V1 HTTP API，直接返回 base64 编码的 MP3 音频。"""
        body = _json.dumps({
            'app': {
                'appid': self.appid,
                'token': 'access_token',
                'cluster': 'volcano_tts',
            },
            'user': {
                'uid': 'abot_robot',
            },
            'audio': {
                'voice_type': TTS_VOICE_TYPE,
                'encoding': 'mp3',
                'speed_ratio': 1.0,
            },
            'request': {
                'reqid': str(_uuid.uuid4()),
                'text': text,
                'operation': 'query',
            },
        })
        req = urllib2.Request(TTS_API_URL, data=body)
        req.add_header('Authorization', 'Bearer;' + self.token)
        req.add_header('Content-Type', 'application/json')
        resp = urllib2.urlopen(req, timeout=10)
        if resp.getcode() != 200:
            rospy.logerr('[DoubaoTTS] HTTP %d: %s', resp.getcode(), resp.read()[:200])
            return

        data = _json.loads(resp.read())
        if data.get('code') != 3000:
            rospy.logerr('[DoubaoTTS] API error code=%d: %s',
                         data.get('code'), data.get('message', ''))
            return

        audio_b64 = data.get('data', '')
        if not audio_b64:
            rospy.logerr('[DoubaoTTS] No audio data in response')
            return

        mp3 = base64.b64decode(audio_b64)
        with tempfile.NamedTemporaryFile(suffix='.mp3', delete=False) as f:
            tmp_path = f.name
        with open(tmp_path, 'wb') as f:
            f.write(mp3)

        duration_ms = data.get('addition', {}).get('duration', '?')
        rospy.loginfo('[DoubaoTTS] Synthesized %d bytes, %sms', len(mp3), duration_ms)

        # 强制输出到扬声器 sink，不被蓝牙耳机劫持（subprocess 防 shell 注入）
        self._play_audio(['mplayer', '-really-quiet', tmp_path])
        os.unlink(tmp_path)


if __name__ == '__main__':
    rospy.init_node('doubao_tts')
    DoubaoTTS()
    rospy.spin()
