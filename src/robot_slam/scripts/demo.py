#!/home/abot/anaconda3/envs/py39/bin/python
# -*- coding: utf-8 -*-
import rospy
import pyaudio
import wave
import os
from funasr import AutoModel
import soundfile
from std_msgs.msg import String
TOP_DIR = os.path.dirname(os.path.abspath(__file__))
music_path = os.path.join(TOP_DIR, "start_record.mp3")
music1_path = os.path.join(TOP_DIR, "end_record.mp3")
os.environ['KMP_DUPLICATE_LIB_OK'] = 'True'

def audio_callback(msg):
    rospy.loginfo("Received audio message, starting recording and recognition")
    start_audio()
    rospy.loginfo("Recording and recognition completed")

def start_audio(time=10, save_file="test.wav"):
    global model
    CHUNK = 1024
    FORMAT = pyaudio.paInt16
    CHANNELS = 2
    RATE = 16000
    RECORD_SECONDS = time  # 需要录制的时间
    WAVE_OUTPUT_FILENAME = save_file  # 保存的文件名

    p = pyaudio.PyAudio()  # 初始化
    rospy.loginfo("ON")
    os.system('mplayer %s' % music_path)
    if os.path.exists(save_file):
        os.remove(save_file)

    stream = p.open(format=FORMAT,
                    channels=CHANNELS,
                    rate=RATE,
                    input=True,
                    frames_per_buffer=CHUNK)  # 创建录音文件
    frames = []

    for i in range(0, int(RATE / CHUNK * RECORD_SECONDS)):
        data = stream.read(CHUNK)
        frames.append(data)  # 开始录音

    rospy.loginfo("OFF")
    os.system('mplayer %s' % music1_path)

    stream.stop_stream()
    stream.close()
    p.terminate()

    wf = wave.open(WAVE_OUTPUT_FILENAME, 'wb')  # 保存
    wf.setnchannels(CHANNELS)
    wf.setsampwidth(p.get_sample_size(FORMAT))
    wf.setframerate(RATE)
    wf.writeframes(b''.join(frames))
    wf.close()

    rospy.loginfo("Starting recognition")
    res = model.generate(input=os.path.join(TOP_DIR, "test.wav"))
    
    result = res[0].get('text','默认值')
    print(result)  
    # 等待直到发布者注册到ROS主节点
    rospy.loginfo("Waiting for publisher to register...")
    while pub1.get_num_connections() == 0 and not rospy.is_shutdown():
        rospy.sleep(0.1)
    while pub2.get_num_connections() == 0 and not rospy.is_shutdown():
        rospy.sleep(0.1)

    # 发布消息
    message = str(result)
    rospy.loginfo("Publishing message: " + message)
    pub1.publish(message)
    pub2.publish(message)
    # 确保消息已经发送出去
    rospy.sleep(0.5)

def audio_subscriber():
    rospy.init_node('audio_subscriber', anonymous=True)
    rospy.Subscriber("audio_topic", String, audio_callback)
    global pub1
    global pub2
    pub1 = rospy.Publisher('chinese_topic', String, queue_size=10)
    pub2 = rospy.Publisher('chinese_topic1', String, queue_size=10)
    rospy.loginfo("Audio subscriber node started")
    rospy.spin()

if __name__ == '__main__':
    model = AutoModel(model=os.path.join(TOP_DIR, "paraformer-zh"),disable_update=True)
    audio_subscriber()
