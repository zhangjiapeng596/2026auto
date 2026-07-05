#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Unit checks for robot_slam/scripts/doubao_asr.py flash ASR request format."""
import base64
import importlib.util
import os
import sys
import tempfile
import types
import unittest


ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
ASR_PATH = os.path.join(ROOT, 'src', 'robot_slam', 'scripts', 'doubao_asr.py')


class DummyPublisher(object):
    def __init__(self, *args, **kwargs):
        pass

    def publish(self, *args, **kwargs):
        pass


class DummyString(object):
    def __init__(self, data=''):
        self.data = data


class FakeResponse(object):
    headers = {
        'X-Api-Status-Code': '20000000',
        'X-Api-Message': 'OK',
        'X-Tt-Logid': 'test-logid',
    }

    status_code = 200

    def json(self):
        return {'result': {'text': '开始比赛'}}


class DoubaoASRFlashTest(unittest.TestCase):
    def setUp(self):
        self._old_modules = dict(sys.modules)

        rospy = types.ModuleType('rospy')
        rospy.Publisher = DummyPublisher
        rospy.loginfo = lambda *args, **kwargs: None
        rospy.logwarn = lambda *args, **kwargs: None
        rospy.logerr = lambda *args, **kwargs: None
        rospy.get_param = lambda *args, **kwargs: False
        sys.modules['rospy'] = rospy

        pyaudio = types.ModuleType('pyaudio')
        pyaudio.paInt16 = 8
        sys.modules['pyaudio'] = pyaudio

        std_msgs = types.ModuleType('std_msgs')
        std_msgs_msg = types.ModuleType('std_msgs.msg')
        std_msgs_msg.String = DummyString
        sys.modules['std_msgs'] = std_msgs
        sys.modules['std_msgs.msg'] = std_msgs_msg

        api_key = types.ModuleType('API_KEY_DOUBAO')
        api_key.SPEECH_APPID = '5728482904'
        api_key.SPEECH_TOKEN = 'test-token'
        api_key.SPEECH_ASR_RESOURCE_ID = 'Speech_Recognition_Seed_AUC2000000786212863330'
        sys.modules['API_KEY_DOUBAO'] = api_key

        spec = importlib.util.spec_from_file_location('doubao_asr_under_test', ASR_PATH)
        self.module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(self.module)

    def tearDown(self):
        sys.modules.clear()
        sys.modules.update(self._old_modules)

    def test_recognize_uses_flash_endpoint_and_base64_audio_data(self):
        captured = {}

        def fake_post(url, headers=None, json=None, timeout=None):
            captured['url'] = url
            captured['headers'] = headers
            captured['json'] = json
            captured['timeout'] = timeout
            return FakeResponse()

        self.module.requests.post = fake_post
        asr = self.module.DoubaoASR()

        with tempfile.NamedTemporaryFile(suffix='.wav', delete=False) as f:
            f.write(b'fake wav bytes')
            path = f.name
        try:
            text = asr._recognize(path)
        finally:
            os.unlink(path)

        self.assertEqual(text, '开始比赛')
        self.assertEqual(
            captured['url'],
            'https://openspeech.bytedance.com/api/v3/auc/bigmodel/recognize/flash')
        self.assertEqual(captured['headers']['X-Api-App-Key'], '5728482904')
        self.assertEqual(captured['headers']['X-Api-Access-Key'], 'test-token')
        self.assertEqual(captured['headers']['X-Api-Resource-Id'], 'Speech_Recognition_Seed_AUC2000000786212863330')
        self.assertEqual(captured['headers']['X-Api-Sequence'], '-1')
        self.assertIn('X-Api-Request-Id', captured['headers'])
        self.assertEqual(captured['json']['user']['uid'], 'abot_robot')
        self.assertEqual(captured['json']['audio']['format'], 'wav')
        self.assertEqual(
            captured['json']['audio']['data'],
            base64.b64encode(b'fake wav bytes').decode('utf-8'))
        self.assertEqual(captured['json']['request']['model_name'], 'bigmodel')


if __name__ == '__main__':
    unittest.main()
