#!/usr/bin/env python3
"""豆包语音 API 烟雾测试 — TTS + ASR 端到端验证。"""
import os, sys, requests, json, uuid, base64, wave, struct, math

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src', 'abot_vlm', 'scripts'))
from API_KEY_DOUBAO import (SPEECH_APPID, SPEECH_TOKEN,
    SPEECH_TTS_RESOURCE_ID, SPEECH_ASR_RESOURCE_ID)

T = SPEECH_TOKEN
A = SPEECH_APPID

ok = 0
total = 0

# ===== TTS =====
print("=" * 55)
print("TTS 语音合成 (V1)")
total += 1
r = requests.post('https://openspeech.bytedance.com/api/v1/tts',
    headers={'Authorization': 'Bearer;' + T, 'Content-Type': 'application/json'},
    json={
        'app': {'appid': A, 'token': 'access_token', 'cluster': 'volcano_tts'},
        'user': {'uid': 'test'},
        'audio': {'voice_type': 'zh_male_M392_conversation_wvae_bigtts', 'encoding': 'mp3', 'speed_ratio': 1.0},
        'request': {'reqid': str(uuid.uuid4()), 'text': '比赛开始', 'operation': 'query'},
    }, timeout=10)
data = r.json()
if data.get('code') == 3000:
    mp3 = base64.b64decode(data['data'])
    print(f"  OK: {len(mp3)} bytes, {data['addition']['duration']}ms")
    ok += 1
else:
    print(f"  FAIL: code={data.get('code')} {data.get('message','')}")

# ===== ASR =====
print("=" * 55)
print("ASR 极速版 (V3 Flash)")
total += 1

# Generate test WAV (pure tone — won't transcribe real text, but validates API connectivity)
wav = '/tmp/asr_smoke.wav'; sr = 16000
with wave.open(wav, 'wb') as wf:
    wf.setnchannels(1); wf.setsampwidth(2); wf.setframerate(sr)
    for i in range(sr * 2):
        wf.writeframes(struct.pack('<h', int(8000 * math.sin(2 * math.pi * 440 * i / sr))))
with open(wav, 'rb') as f:
    audio = f.read()
os.unlink(wav)

r = requests.post('https://openspeech.bytedance.com/api/v3/auc/bigmodel/recognize/flash',
    headers={
        'X-Api-App-Key': A,
        'X-Api-Access-Key': T,
        'X-Api-Resource-Id': 'volc.bigasr.auc_turbo',
        'X-Api-Request-Id': str(uuid.uuid4()),
        'X-Api-Sequence': '-1',
        'Content-Type': 'application/json',
    },
    json={
        'user': {'uid': 'test'},
        'audio': {'format': 'wav', 'data': base64.b64encode(audio).decode()},
        'request': {'model_name': 'bigmodel', 'enable_itn': True, 'enable_punc': True},
    }, timeout=10)
status = r.headers.get('X-Api-Status-Code', '')
if status == '20000000':
    data = r.json()
    text = data.get('result', {}).get('text', '') if isinstance(data.get('result'), dict) else data.get('text', '')
    print(f"  OK: status={status} text='{text}'")
    ok += 1
else:
    print(f"  FAIL: http={r.status_code} status={status} msg={r.headers.get('X-Api-Message','?')}")

# ===== Summary =====
print("=" * 55)
print(f"Result: {ok}/{total} passed")
print("=" * 55)
