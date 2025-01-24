#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String

import spacy
import pyaudio
import webrtcvad
import wave
import whisper
import openai
import time
import threading
import queue
import signal
import os
import io
import pydub
from playsound import playsound
import numpy as np
from zhconv import convert
from typing import Dict, List
import datetime

# ------------------- 配置部分 -------------------
SAMPLE_RATE = 16000          # WebRTC VAD 支持: 8k/16k/32k/48k
FRAME_DURATION_MS = 10       # 帧时长(10/20/30ms)
CHANNELS = 1                 # 单声道
VAD_MODE = 3                 # 0~3, 值越大越敏感
SILENCE_DURATION = 1.0       # 连续多少秒静音视为语音结束
WHISPER_MODEL_NAME = "small"  # 可选: "tiny", "base", "small", "medium", "large"
USE_CUDA = True             # True 则尝试在 GPU 上推理

# ------------------- 全局变量 -------------------
audio_queue = queue.Queue()     # 存放音频段给识别线程
parsed_queue = queue.Queue()    # 存放解析任务的队列
play_tts_queue = queue.Queue()  # 新增用于播放音频的队列
stop_event = threading.Event()  # 控制录音/识别线程结束
vad = webrtcvad.Vad(VAD_MODE)
language = ""
nlp = spacy.load("ja_core_news_md")
voice_mode = "chat"  # Initialize voice mode variable
tts_stop = False
MAX_CONTINUE_SEC = 5000 #ms

# 录音状态
in_speech = False
silent_frame_count = 0
recorded_frames = []
p = pyaudio.PyAudio()


# 计算帧大小
frame_samples = int(SAMPLE_RATE * FRAME_DURATION_MS / 1000)  # 例如 16kHz * 20ms = 320
frame_bytes = frame_samples * 2  # 16位采样 -> 每样本2字节

#定义音频响应数据字典
audo_path = "/home/mikuni/catkin_ws/src/amr_ros/resource/"
voice_resp: Dict[str, Dict[str,str]] = {
    "ja": {"resp_wakeup": audo_path + "resp_wakeup_ja.mp3",
           "resp_start_to_run": audo_path + "resp_start_to_run_ja.mp3",
           "resp_stop_and_wait": audo_path + "resp_stop_and_wait_ja.mp3",
           "resp_chat_over": audo_path + "resp_chat_over_ja.mp3"},
    "zh": {"resp_wakeup": audo_path + "resp_wakeup_zh.mp3",
           "resp_start_to_run": audo_path + "resp_start_to_run_zh.mp3",
           "resp_stop_and_wait": audo_path + "resp_stop_and_wait_zh.mp3",
           "resp_chat_over": audo_path + "resp_chat_over_zh.mp3"},
}

wakeup_dict: Dict[str, List[str]] = {
    "ja": ["ミク","みく","ミック","みっく","ミクロボ","みくろぼ"],
    "zh": ["小度", "小杜","小肚","小渡","小兔"],
}

chat_over_dict: Dict[str, List[str]] = {
    "ja": ["会話終了","会話中止","ストップ"],
    "zh": ["停止对话","结束对话","对话结束","会话结束","结束会话","终止对话","中断对话"],
}

#save log
current_time_str = "/home/mikuni/catkin_ws/logs/voice_" + datetime.datetime.now().strftime("%Y%m%d_%H_%M_%S")


def update_voice_mode(msg):
    global voice_mode
    global tts_stop
    voice_mode = msg.data

    if voice_mode != "chat":
        tts_stop = True
    rospy.loginfo(f"Updated voice_mode: {voice_mode}")

# 初始化ROS订阅者
robot_cmd_subscriber = rospy.Subscriber("/voice_set_mode", String, callback=update_voice_mode)
# 初始化ROS发布者
robot_cmd_publisher = rospy.Publisher("/voice_ctrl_cmd", String, queue_size=10)

# 从环境变量中读取OpenAI API密钥
openai.api_key = os.getenv('OPENAI_API_KEY')


def signal_handler(sig, frame):
    """捕获 Ctrl+C / kill 信号，用于安全退出。"""
    rospy.loginfo("检测到退出信号，正在停止节点...")
    stop_event.set()

def wrap_pcm_to_wav(pcm_data: bytes, sample_rate: int, channels: int) -> io.BytesIO:
    """
    将裸 PCM 数据封装为 WAV 格式并存储到 BytesIO，方便后续识别。
    """
    wav_io = io.BytesIO()
    with wave.open(wav_io, "wb") as wf:
        wf.setnchannels(channels)
        wf.setsampwidth(2)  # 16-bit
        wf.setframerate(sample_rate)
        wf.writeframes(pcm_data)
    wav_io.seek(0)
    return wav_io

def whisper_worker(pub):
    """
    识别线程：不断从 audio_queue 中取出音频片段，用 Whisper 识别，
    并将结果发布到 ROS topic。
    """
    rospy.loginfo("识别线程已启动。正在加载 Whisper 模型: %s", WHISPER_MODEL_NAME)
    device = "cuda" if USE_CUDA else "cpu"
    model = whisper.load_model(WHISPER_MODEL_NAME, device=device)
    rospy.loginfo("Whisper 模型加载完毕，等待音频数据...")
    global language

    while not rospy.is_shutdown() and not stop_event.is_set():
        try:
            pcm_data = audio_queue.get(timeout=0.5)  # 最多等0.5秒
        except queue.Empty:
            continue

        # 若取到 None，表示主线程要结束
        if pcm_data is None:
            rospy.loginfo("识别线程收到结束信号，退出。")
            break

        # 将PCM封装成WAV并识别
        wav_buf = wrap_pcm_to_wav(pcm_data, SAMPLE_RATE, CHANNELS)
        wav_buf.seek(0)
        audio_segment = pydub.AudioSegment.from_file(wav_buf, format="wav")
        samples = np.array(audio_segment.get_array_of_samples())
        if audio_segment.sample_width == 2: # 16-bit
            audio_data = samples.astype(np.float32) / 32768.0
        else:
            pass

        result = model.transcribe(audio_data, language=language,fp16=USE_CUDA,
                no_speech_threshold=0.1)

        text = result["text"]
        if (text == "") or (len(result['segments']) == 0) or (result['segments'][0]['no_speech_prob'] > 0.5):
            pass
        else:
            if language == "zh":
                text = convert(text, 'zh-hans') #转换为中文简体

            rospy.loginfo("Whisper 识别结果: %s", text)
            #debug_ryu
            #write log
            with open(current_time_str, 'a', encoding='utf-8') as f:
                time_stemp = datetime.datetime.now().strftime("%H_%M_%S")
                f.write(time_stemp)
                formatted_line = ":%s\n" % (text)
                f.write(formatted_line)

            # 发布到 ROS topic
            pub.publish(text)
            parsed_queue.put(text)  # 放入解析队列

        audio_queue.task_done()

    rospy.loginfo("识别线程结束。")

def parse_command(command):
    """
    分割解析指令
    """
    doc = nlp(command)

    parsed_command = {
        "action": None,
        "value": None,
        "unit": None,
        "object": None
    }

    for token in doc:
        if token.pos_ == "VERB":  # 如果是动词，识别为动作
            parsed_command["action"] = token.text
        elif token.pos_ == "NUM":  # 如果是数词，识别为值
            parsed_command["value"] = token.text
        elif token.pos_ == "NOUN":  # 如果是名词，可能是单位或动作对象
            parsed_command["unit"] = token.text
            parsed_command["object"] = token.text

    return parsed_command

def execute_command(command):
    """
    执行控制指令，并使用TTS报告结果
    """
    #play music
    start_robot = ["走っ","はしっ","スタート","走れ","はしれ","動け","うごけ","开始","前进","出发"]
    stop_robot = ["止まっ","とまっ","ストップ","止まれ","とまれ","待て","まて","待っ","まっ","停车"]

    resp_cmd = ""
    if command['action'] in start_robot:
        response = voice_resp[language]["resp_start_to_run"]
        resp_cmd = "start"
        robot_cmd_publisher.publish(resp_cmd)
    elif (command['action'] in stop_robot) or ((language == "zh") and (command['object'] in stop_robot)):
        response = voice_resp[language]["resp_stop_and_wait"]
        resp_cmd = "stop"
        robot_cmd_publisher.publish(resp_cmd)
    else:
        pass

    if len(resp_cmd) > 0:
        print(f"language={language}, command resp:{response}")
        playsound(response)
    else:
        print(f"Command not recognized: {command['action']}")

def chat_with_gpt(text):
    """
    将文本发送给ChatGPT并获取响应
    """
    try:
        response = openai.chat.completions.create(
            model="gpt-4o-mini",
            messages=[
                {
                    "role": "user",
                    "content": text,
                },
            ],
        )
        print(response.choices[0].message.content)
        return response.choices[0].message.content
    except openai.error.OpenAIError as e:
        rospy.logerr(f"OpenAI API error: {e}")
        return ""


def parser_worker():
    """
    解析线程：从 parsed_queue 中取出识别文本，并进行解析。
    """
    chat_start=False
    is_continue_chat = False
    global tts_stop
    start_time = time.time()

    rospy.loginfo("解析线程已启动。")
    while not rospy.is_shutdown() and not stop_event.is_set():

        try:
            text = parsed_queue.get(timeout=0.5)  # 最多等0.5秒
        except queue.Empty:
            continue

        if text is None:
            rospy.loginfo("解析线程收到结束信号，退出。")
            break

        # 在此处添加解析逻辑
        if voice_mode == "chat":
            #检查是否要结束对话
            if chat_start == True or is_continue_chat == True:
                for word in chat_over_dict[language]:
                    if word in text:
                        chat_start = False
                        is_continue_chat = False
                        tts_stop = True
                        playsound(voice_resp[language]["resp_chat_over"])
                        break

            #检查是否唤醒词
            for word in wakeup_dict[language]:
                if word in text:
                    chat_start = True
                    is_continue_chat = True
                    tts_stop = True
                    start_time = time.time()
                    playsound(voice_resp[language]["resp_wakeup"])
                    break

            if chat_start == True:
                chat_start = False
                continue
            if is_continue_chat == True:
                #保存响应数据用于TTS线程播放
                play_tts_queue.put(text)
                start_time = time.time()

        elif voice_mode == "control":
            parsed_text = parse_command(text)
            execute_command(parsed_text)
        else:
            pass

        if int((time.time() - start_time) * 1000) > MAX_CONTINUE_SEC:
            is_continue_chat = False

        parsed_queue.task_done()

    rospy.loginfo("解析线程结束。")

def play_tts_worker():
    """
    播放音频线程：不断从 play_tts_queue 中取出音频数据，并播放。
    """
    rospy.loginfo("播放音频线程已启动。")
    global tts_stop

    player_stream = p.open(format=pyaudio.paInt16,
                            channels=1,
                            rate=24000,
                            output=True)

    while not rospy.is_shutdown() and not stop_event.is_set():
        try:
            audio_tts_data = play_tts_queue.get(timeout=0.5)  # 最多等0.5秒
        except queue.Empty:
            continue

        if audio_tts_data is None:
            rospy.loginfo("播放音频线程收到结束信号，退出。")
            break

        # 播放音频
        try:
            resp_text = chat_with_gpt(audio_tts_data)
            if resp_text == "":
                print("TTS API Error. Get chat response failed.")
            else:
                tts_stop = False
                with openai.audio.speech.with_streaming_response.create(
                    model="tts-1",
                    voice="nova", #"alloy",
                    response_format="pcm",  # similar to WAV, but without a header chunk at the start.
                    input=resp_text,
                ) as response:
                    for chunk in response.iter_bytes(chunk_size=512):
                        if tts_stop == True: #中断播放，清空缓存队列
                            while not play_tts_queue.empty():
                                play_tts_queue.get()
                            break
                        player_stream.write(chunk)

        except Exception as e:
            rospy.logerr(f"播放音频时出错: {e}")

        play_tts_queue.task_done()

    rospy.loginfo("播放音频线程结束。")
    player_stream.stop_stream()
    player_stream.close()


def audio_callback(in_data, frame_count, time_info, status_flags):
    """PyAudio 回调函数：实时处理音频帧，利用 VAD 做语音段切分。"""
    global in_speech, silent_frame_count, recorded_frames

    if stop_event.is_set() or rospy.is_shutdown():
        # 如果主线程发出停止信号，则结束
        return (None, pyaudio.paComplete)

    # 如果帧大小跟预期不一致，就跳过这帧
    if len(in_data) != frame_bytes:
        return (None, pyaudio.paContinue)

    # 使用 VAD 判断是否语音
    is_speech = vad.is_speech(in_data, SAMPLE_RATE)

    if is_speech:
        if not in_speech:
            rospy.loginfo("[录音回调] 检测到语音，开始新段落...")
            in_speech = True
        recorded_frames.append(in_data)
        silent_frame_count = 0
    else:
        if in_speech:
            # 处于说话状态，这帧无语音
            recorded_frames.append(in_data)  # 可选：把这帧也加入
            silent_frame_count += 1
            needed_silence_frames = int(SILENCE_DURATION * 1000 / FRAME_DURATION_MS)
            if silent_frame_count >= needed_silence_frames:
                rospy.loginfo("[录音回调] 检测到静音，结束本段录音，放入识别队列。")
                end_segment()
        else:
            # 未进入说话状态，不存音频，直接等待
            pass

    return (None, pyaudio.paContinue)

def end_segment():
    """将当前 recorded_frames 拼成一段音频，放到队列，重置状态。"""
    global in_speech, silent_frame_count, recorded_frames

    pcm_data = b"".join(recorded_frames)
    audio_queue.put(pcm_data)  # 放入队列给识别线程
    in_speech = False
    silent_frame_count = 0
    recorded_frames = []

def main():
    # ROS节点初始化
    rospy.init_node("speech_recognition_node", anonymous=True)
    rospy.loginfo("启动 speech_recognition_node...")

    # 注册信号处理（捕获 Ctrl+C）
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    # 建立一个发布器，将识别文本发布到 /voice_result
    pub = rospy.Publisher("/voice_result", String, queue_size=10)    
    # 获取语言参数
    global language
    global nlp
    language = rospy.get_param("~language", "ja")
    if language == "ja":
        nlp = spacy.load("ja_core_news_md")
    elif language == "zh":
        nlp = spacy.load("zh_core_web_md")
    else:
        pass
    # 启动识别线程
    worker_thread = threading.Thread(target=whisper_worker, args=(pub,))
    worker_thread.start()

    # 启动解析线程
    parser_thread = threading.Thread(target=parser_worker)
    parser_thread.start()

    # 启动播放音频线程
    play_tts_thread = threading.Thread(target=play_tts_worker)
    play_tts_thread.start()

    # 打开 PyAudio 流，回调模式
    stream = p.open(
        format=pyaudio.paInt16,
        channels=CHANNELS,
        rate=SAMPLE_RATE,
        input=True,
        frames_per_buffer=frame_samples,
        stream_callback=audio_callback
    )
    stream.start_stream()
    rospy.loginfo("开始并发录音和识别，按 Ctrl+C 退出...")

    # 主循环：只要 ROS 没关闭、stop_event 未触发，就保持活跃
    while not rospy.is_shutdown() and not stop_event.is_set():
        time.sleep(0.1)

    global tts_stop
    tts_stop = True
    # 收尾：停止录音
    rospy.loginfo("停止录音流...")
    stream.stop_stream()
    stream.close()

    # 若还有一段正在录音中，也强制结束
    if in_speech and len(recorded_frames) > 0:
        rospy.loginfo("录音结束前，将最后一段放入识别队列...")
        end_segment()

    # 给识别线程发送结束标志(None)
    audio_queue.put(None)
    worker_thread.join()

    # 给解析线程发送结束标志(None)
    parsed_queue.put(None)
    parser_thread.join()

    # 给播放线程发送结束标志(None)
    play_tts_queue.put(None)
    play_tts_thread.join()
    p.terminate()

    rospy.loginfo("节点退出。")

if __name__ == "__main__":
    main()
