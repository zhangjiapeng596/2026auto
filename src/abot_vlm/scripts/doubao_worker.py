#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""豆包 VLM Worker — 纯 Python 3.9 进程，不依赖 ROS。
由 vlm_bridge.py (py2.7) 通过 subprocess 调用。
输入: 命令行参数 --image PATH --prompt PROMPT (或 --prompt-file PATH)
输出: JSON 结果写到 stdout，错误写到 stderr
退出码: 0=成功, 1=失败
"""
from volcenginesdkarkruntime import Ark
import os, sys, json, re, base64, argparse

# === API Key ===
DOUBAO_KEY = os.environ.get('DOUBAO_KEY')
if not DOUBAO_KEY:
    try:
        from API_KEY_DOUBAO import DOUBAO_KEY
    except Exception:
        DOUBAO_KEY = None

DEFAULT_PROMPT = (
    '你是一个自主机器人比赛的视觉任务识别系统。你的任务是：'
    '\n1. **仔细阅读** 图中围栏内侧（距地面约20cm高度）贴着的任务卡片上的文字。'
    '任务卡片包含一道数学或逻辑题目（中文或数字）。'
    '\n2. **逐步求解** 这道题。写出你的推理过程。'
    '\n3. **将题目的最终答案数值作为目标网格编号** 输出。'
    '答案必须是以下9个数字之一：31, 32, 33, 40, 41, 42, 49, 50, 51。'
    '如果计算出的答案恰好不在这个范围内，选择最接近的合法值。'
    '如果完全无法读出文字或无法求解，confidence 设为 0。'
    '\n**置信度判断规则：**'
    '0.9-1.0：文字清晰可读，题目完整可解，答案确定；'
    '0.6-0.89：文字基本可读，题目基本可辨，答案合理推测；'
    '0.0-0.59：文字模糊不清、无法识别、或无法确定答案。'
    '\n**回复格式必须是严格的 JSON（不要包含 markdown 代码块标记）：**'
    '{"target_cell": <整数>, "content": "<读取到的题目内容和求解过程>", "confidence": <0到1的浮点数>}'
    '\n**示例：** 图中卡片内容为"5×7+2=?"，求解得37。'
    '应输出：{"target_cell": 37, "content": "卡片内容为5×7+2=37", "confidence": 0.95}'
)


def call_doubao(image_path, prompt):
    """调用豆包 Vision API，返回原始文本。"""
    with open(image_path, 'rb') as f:
        image_b64 = 'data:image/jpeg;base64,' + base64.b64encode(f.read()).decode('utf-8')

    client = Ark(
        base_url="https://ark.cn-beijing.volces.com/api/v3",
        api_key=DOUBAO_KEY
    )
    response = client.chat.completions.create(
        model="doubao-1-5-vision-pro-32k-250115",
        messages=[{
            "role": "user",
            "content": [
                {"type": "text", "text": prompt},
                {"type": "image_url", "image_url": {"url": image_b64}},
            ]
        }],
    )
    return response.choices[0].message.content.strip()


VALID_TARGET_CELLS = {31, 32, 33, 40, 41, 42, 49, 50, 51}


def parse_result(raw_text):
    """解析 VLM 返回文本为结构化 JSON。校验 target_cell 必须在合法范围内。"""
    target_cell, content, confidence = None, str(raw_text), 0.0
    try:
        m = re.search(r'\{.*\}', raw_text, re.DOTALL)
        if m:
            data = json.loads(m.group(0))
            target_cell = data.get('target_cell')
            content = data.get('content', content)
            confidence = float(data.get('confidence', 0.0))
            if target_cell is not None:
                target_cell = int(target_cell)
                if target_cell not in VALID_TARGET_CELLS:
                    content = 'invalid_cell_{}: {}'.format(target_cell, content)
                    target_cell = None
                    confidence = 0.0
    except (ValueError, TypeError):
        target_cell, confidence = None, 0.0
    return {'target_cell': target_cell, 'content': content, 'confidence': confidence}


def main():
    parser = argparse.ArgumentParser(description='Doubao VLM Worker')
    parser.add_argument('--image', required=True, help='Path to image file')
    parser.add_argument('--prompt', default=None, help='Prompt text')
    parser.add_argument('--prompt-file', default=None, help='Read prompt from file')
    parser.add_argument('--image-id', default='unknown', help='Image identifier')
    args = parser.parse_args()

    if DOUBAO_KEY is None:
        print(json.dumps({'error': 'DOUBAO_KEY not set'}), file=sys.stderr)
        sys.exit(1)

    prompt = args.prompt or DEFAULT_PROMPT
    if args.prompt_file and os.path.isfile(args.prompt_file):
        with open(args.prompt_file, 'r') as f:
            prompt = f.read().strip()

    try:
        raw = call_doubao(args.image, prompt)
        result = parse_result(raw)
        result['image_id'] = args.image_id
        result['raw_response'] = raw
        print(json.dumps(result, ensure_ascii=False))
        sys.exit(0)
    except Exception as e:
        print(json.dumps({
            'target_cell': None, 'content': 'error: {}'.format(str(e)),
            'confidence': 0.0, 'image_id': args.image_id
        }, ensure_ascii=False))
        sys.exit(1)


if __name__ == '__main__':
    main()
