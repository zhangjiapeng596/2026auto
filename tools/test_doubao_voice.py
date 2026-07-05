#!/usr/bin/env python3
"""WSL 环境下测试豆包 ASR/TTS API 连通性。"""

import os, sys, requests, json, uuid, base64, time, wave, struct, math

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src', 'abot_vlm', 'scripts'))
from API_KEY_DOUBAO import (SPEECH_APPID, SPEECH_TOKEN,
    SPEECH_TTS_RESOURCE_ID, SPEECH_ASR_RESOURCE_ID)

AUTH = 'Bearer; ' + SPEECH_TOKEN


def test_tts():
    """测试豆包 TTS API (语音合成大模型-字符版)。"""
    print("\n=== TTS 语音合成 ===")
    h = {'Authorization': AUTH, 'Resource-Id': SPEECH_TTS_RESOURCE_ID, 'Content-Type': 'application/json'}
    body = {
        'appid': SPEECH_APPID,
        'reqid': str(uuid.uuid4()),
        'text': '比赛开始',
        'format': 'mp3',
        'voice_type': 'BV701_streaming',
        'sample_rate': 24000,
    }
    print(f"  Auth: {AUTH[:30]}...")
    print(f"  Text: {body['text']}")
    r = requests.post('https://openspeech.bytedance.com/api/v1/tts_async/submit', headers=h, json=body, timeout=10)
    print(f"  Status: {r.status_code}")
    print(f"  Body: {r.text[:300]}")
    if r.status_code != 200:
        return False
    data = r.json()
    tid = data.get('task_id', '')
    if not tid:
        return False
    print(f"  task_id: {tid}")
    for i in range(30):
        time.sleep(0.3)
        q = requests.get('https://openspeech.bytedance.com/api/v1/tts_async/query',
                         headers=h, params={'appid': SPEECH_APPID, 'task_id': tid}, timeout=5)
        if q.status_code == 200:
            qd = q.json()
            s = qd.get('status', '')
            print(f"  poll {i}: {s}")
            if s == 'success':
                url = qd.get('audio_url', '')
                print(f"  audio_url: {url[:80]}")
                print("  TTS PASS")
                return True
    print("  TTS TIMEOUT")
    return False


def test_asr():
    """测试豆包 ASR API (录音文件识别大模型-标准版)。"""
    print("\n=== ASR 语音识别 ===")
    wav = '/tmp/test_speech.wav'
    sr = 16000
    with wave.open(wav, 'wb') as wf:
        wf.setnchannels(1); wf.setsampwidth(2); wf.setframerate(sr)
        for i in range(sr * 3):
            v = int(16000 * math.sin(2 * math.pi * 440 * i / sr))
            wf.writeframes(struct.pack('<h', v))
    with open(wav, 'rb') as f:
        audio = f.read()
    os.unlink(wav)

    # 试多种 ASR body 格式
    variants = [
        {'appid': SPEECH_APPID},
        {'app_id': SPEECH_APPID},
        {},
    ]
    for v in variants:
        h = {'Authorization': AUTH, 'Resource-Id': SPEECH_ASR_RESOURCE_ID, 'Content-Type': 'application/json'}
        body = {
            'reqid': str(uuid.uuid4()),
            'audio': base64.b64encode(audio).decode('utf-8'),
            'audio_format': 'wav',
            'sample_rate': sr,
        }
        body.update(v)
        r = requests.post('https://openspeech.bytedance.com/api/v1/asr', headers=h, json=body, timeout=10)
        print(f"  ASR with {v}: {r.status_code} {r.text[:200]}")
        if r.status_code == 200:
            print("  ASR PASS")
            return True
    return False


if __name__ == '__main__':
    print("豆包语音 API 测试")
    print(f"APPID: {SPEECH_APPID}  TOKEN: {SPEECH_TOKEN[:20]}...")
    tts_ok = test_tts()
    asr_ok = test_asr()
    print(f"\n结果: TTS={'PASS' if tts_ok else 'FAIL'}  ASR={'PASS' if asr_ok else 'FAIL'}")
