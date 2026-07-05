#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""结构化日志：按 run_YYYYMMDD_HHMMSS 组织，记录状态机跳转、识别、导航、播报结果。"""
import sys
reload(sys)
sys.setdefaultencoding('utf-8')
import os
import json
import time
import threading
from datetime import datetime


class MissionLogger(object):
    """比赛日志管理器。线程安全。"""

    def __init__(self, base_dir=None):
        if base_dir is None:
            workspace = os.environ.get('WS_PATH', os.path.expanduser('~/3X2KY2'))
            base_dir = os.path.join(workspace, 'logs')
        run_id = datetime.now().strftime('run_%Y%m%d_%H%M%S')
        self.run_dir = os.path.join(base_dir, run_id)
        try:
            os.makedirs(self.run_dir)
        except OSError:
            # Directory already exists (race condition with other nodes)
            if not os.path.isdir(self.run_dir):
                raise
        self._lock = threading.Lock()

    def _append(self, filename, record):
        record['_timestamp'] = time.time()
        with self._lock:
            filepath = os.path.join(self.run_dir, filename)
            with open(filepath, 'a') as f:
                f.write(json.dumps(record, ensure_ascii=False) + '\n')

    def log_state_transition(self, from_state, to_state, reason=''):
        self._append('mission_log.jsonl', {
            'type': 'state_transition',
            'from': from_state,
            'to': to_state,
            'reason': reason,
        })

    def log_perception(self, result):
        self._append('perception_results.jsonl', {
            'type': 'perception',
            'result': result,
        })

    def log_navigation(self, goal, arrived_pose, success):
        self._append('navigation_goals.jsonl', {
            'type': 'navigation',
            'goal': goal,
            'arrived_pose': arrived_pose,
            'success': success,
        })

    def log_voice(self, text, status):
        self._append('voice_events.jsonl', {
            'type': 'voice',
            'text': text,
            'status': status,
        })

    def log_system(self, event, detail=''):
        self._append('system_status.jsonl', {
            'type': 'system',
            'event': event,
            'detail': detail,
        })
