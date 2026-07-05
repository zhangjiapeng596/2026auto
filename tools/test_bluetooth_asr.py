#!/usr/bin/env python3
"""蓝牙麦克风 → 豆包 ASR 全链路测试。倒计时后录音4秒，输出波形+识别结果。"""
import subprocess, wave, struct, base64, json, uuid, requests, os, sys, time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src', 'abot_vlm', 'scripts'))
from API_KEY_DOUBAO import SPEECH_APPID, SPEECH_TOKEN, SPEECH_ASR_RESOURCE_ID

# 1. 找蓝牙源
bt = None
for line in subprocess.check_output(['pactl', 'list', 'short', 'sources']).decode().split('\n'):
    if 'bluez_source' in line:
        bt = line.split()[1]
        break
print('[1/4] BT source:', bt or 'NOT FOUND — will use default')

# 2. 倒计时
print('[2/4] 倒计时...')
for i in [3, 2, 1]:
    print('  %d...' % i, flush=True)
    time.sleep(1)
print('  ▶ 说话！', flush=True)

# 3. 录音 4 秒
cmd = ['parec', '--format=s16le', '--rate=8000', '--channels=1']
if bt:
    cmd += ['--device=' + bt]
proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
raw = b''
deadline = time.time() + 4
while time.time() < deadline:
    chunk = proc.stdout.read(4096)
    if not chunk:
        break
    raw += chunk
proc.terminate()
proc.wait()

n = len(raw) // 2
samples = struct.unpack('<%dh' % n, raw[:n * 2])
mx = max(abs(s) for s in samples)
print('[3/4] 波形: %d samples, Max amplitude = %d  %s' %
      (n, mx, '✅ 有声音' if mx > 100 else '❌ 静音'))

# 4. 存 WAV → ASR API
with wave.open('/tmp/asr_test.wav', 'wb') as wf:
    wf.setnchannels(1)
    wf.setsampwidth(2)
    wf.setframerate(8000)
    wf.writeframes(raw)

with open('/tmp/asr_test.wav', 'rb') as f:
    audio_b64 = base64.b64encode(f.read()).decode()

print('[4/4] 调用 ASR...', flush=True)
resp = requests.post(
    'https://openspeech.bytedance.com/api/v3/auc/bigmodel/recognize/flash',
    headers={
        'X-Api-App-Key': SPEECH_APPID,
        'X-Api-Access-Key': SPEECH_TOKEN,
        'X-Api-Resource-Id': SPEECH_ASR_RESOURCE_ID,
        'X-Api-Request-Id': str(uuid.uuid4()),
        'X-Api-Sequence': '-1',
        'Content-Type': 'application/json',
    },
    json={
        'user': {'uid': 'abot_test'},
        'audio': {'format': 'wav', 'data': audio_b64},
        'request': {'model_name': 'bigmodel', 'enable_itn': True, 'enable_punc': True},
    },
    timeout=10,
)

sc = resp.headers.get('X-Api-Status-Code', '')
msg = resp.headers.get('X-Api-Message', '')
print('  Status: %s | %s' % (sc, msg))

if sc == '20000000':
    data = resp.json()
    result = data.get('result', {})
    text = result.get('text', '') if isinstance(result, dict) else data.get('text', '')
    print('  识别结果: %s' % repr(text))
    if text and '开始比赛' in text:
        print('  ✅ 唤醒词匹配成功！')
    else:
        print('  ❌ 未匹配"开始比赛"')
else:
    print('  ❌ API 调用失败')

os.unlink('/tmp/asr_test.wav')
