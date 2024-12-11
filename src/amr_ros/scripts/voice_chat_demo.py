#!/usr/bin/env python3
import torch
import whisper
import spacy
import rospy
from std_msgs.msg import String
from zhconv import convert
from openai import OpenAI
import time
import os
import speech_recognition as sr
import pyaudio
from typing import Dict, List

# 检查是否支持CUDA
if torch.cuda.is_available():
    device = "cuda"
    rospy.loginfo("Using CUDA (GPU)")
else:
    device = "cpu"
    rospy.loginfo("Using CPU")

# 初始化Whisper模型
model = whisper.load_model("small", device=device)

# 初始化spaCy
# nlp = spacy.load("zh_core_web_md")
nlp = spacy.load("ja_core_news_md")
lang_cur = "ja"

# ROS话题名称
music_cmd_pub_topic = "/music_cmd"
robot_cmd_pub_topic = "/robot_ctrl_cmd"
# 初始化ROS发布者
music_cmd_publisher = rospy.Publisher(music_cmd_pub_topic, String, queue_size=10)
robot_cmd_publisher = rospy.Publisher(robot_cmd_pub_topic, String, queue_size=10)


# 从环境变量中读取OpenAI API密钥
openai = OpenAI()
openai.api_key = os.getenv('OPENAI_API_KEY')

# 初始化麦克风识别器
recognizer = sr.Recognizer()

def recognize_speech(time_out = -1):
    """
    使用Whisper模型进行语音识别
    """
    recognizer.energy_threshold = 10000
    print("请说话...")
    with sr.Microphone() as source:
        while True:
            #recognizer.adjust_for_ambient_noise(source, duration=1)
            if(time_out == -1):
                audio = recognizer.listen(source)
            else:
                try:
                    audio = recognizer.listen(source, timeout = time_out)
                except sr.WaitTimeoutError:
                    return ""
                except sr.UnknownValueError:
                    return ""

            with open("audio_clip.wav", "wb") as f:
                f.write(audio.get_wav_data())

            # 将音频保存为临时文件
            result = model.transcribe("audio_clip.wav")
            if(len(result['segments']) == 0):
                continue

            if ((result['text'].strip() == 'Thank you.') and (result['language'] == 'en') and (result['segments'][len(result['segments']) - 1]['end'] == 2.0)):
                continue

            if((result['language'] == 'en') or (result['language'] == 'zh') or  (result['language'] == 'ja')):
                ret_str = result['text']
                if(result['language'] == 'zh'):
                    ret_str = convert(result['text'], 'zh-hans')
                if(time_out == -1):
                    return ret_str, result['language']
                else:
                    return ret_str


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

def execute_command(command, lang = 'ja'):
    """
    执行控制指令，并使用TTS报告结果
    """
    #play music
    start_robot = ["开始","前进","出发","進ん","進め","出発","しゅっぱつ","すすん","すすめ","スタット"]
    stop_robot = ["停车","停止","停","ストップ","止まっ","とまっ","止まれ","とまれ"]
    start_music = ["播放","打开","かけ","掛け"]
    stop_music = ["关闭","停止","暂停","消し","けし","止め","とめ"]
    oper_object = ["音乐","音楽","ミュージック"]

    resp_cmd = ""
    if(lang == 'ja'):
        if command['action'] in  start_robot:
            response = "進みます"
            #publish
            resp_cmd = "start"
            robot_cmd_publisher.publish(resp_cmd)
        elif command['action'] in stop_robot:
            response = "止まります."
            #publish
            resp_cmd = "stop"
            robot_cmd_publisher.publish(resp_cmd)
        elif command['action'] in start_music:
            if command['object'] in oper_object:
                #start music
                response = "音楽を掛けます"
                resp_cmd = "play"
                music_cmd_publisher.publish(resp_cmd)
        elif command['action'] in stop_music:
            if command['object'] in oper_object:
                #stop music
                response = "音楽を止めます"
                resp_cmd = "stop"
                music_cmd_publisher.publish(resp_cmd)
        else:
            response = "コマンドを分かりません"

    if(lang == 'zh'):
        if command['action'] in  start_robot:
            response = "开始前进了"
            #publish
            resp_cmd = "start"
            robot_cmd_publisher.publish(resp_cmd)
        elif ((command['action'] in stop_robot) and (command['object'] not in oper_object)) or (command['object'] in stop_robot):
            response = "停车了"
            #publish
            resp_cmd = "stop"
            robot_cmd_publisher.publish(resp_cmd)
        elif command['action'] in start_music:
            if command['object'] in oper_object:
                #start music
                response = "开始播放音乐了"
                resp_cmd = "play"
                music_cmd_publisher.publish(resp_cmd)
        elif command['action'] in stop_music:
            if command['object'] in oper_object:
                #stop music
                response = "停止播放音乐了"
                resp_cmd = "stop"
                music_cmd_publisher.publish(resp_cmd)
        else:
            response = "指令没有被识别。"

    # 使用TTS播放执行结果
    text_to_speech(response)

def chat_with_gpt(text):
    """
    将文本发送给ChatGPT并获取响应
    """
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

def text_to_speech(text):
    player_stream = pyaudio.PyAudio().open(format=pyaudio.paInt16, channels=1, rate=24000, output=True)

    start_time = time.time()

    with openai.audio.speech.with_streaming_response.create(
        model="tts-1",
        voice="nova", #"alloy",
        response_format="pcm",  # similar to WAV, but without a header chunk at the start.
        input=text,
    ) as response:
        print(f"Time to first byte: {int((time.time() - start_time) * 1000)}ms")
        for chunk in response.iter_bytes(chunk_size=1024):
            player_stream.write(chunk)

    print(f"Done in {int((time.time() - start_time) * 1000)}ms.")

def find_wakeup_word(text):
    wakeup_dict: Dict[str, List[str]] = {
        "zh": ["小度小度", "小杜小杜","小肚小肚","小渡小渡"],
        "ja": ["ミクロさん", "みくろさん", "ミクロサン","ミクロサウン"],
    }

    if (text.strip() in wakeup_dict["ja"]) or (text.strip() in wakeup_dict["zh"]):
        return True

    return False

def main():

    rospy.init_node("speech_recognition_node", anonymous=True)
    rate = rospy.Rate(10)  # 10Hz
    rospy.loginfo("Speech Recognition Node Started.")

    mode = "control"  # 初始模式为控制模式
    global nlp
    global lang_cur

    while not rospy.is_shutdown():
        print("Listening for wakeup word:")
        text, lang = recognize_speech()
        print(f"Recognized: {text}")
        #check wakeup word
        if(find_wakeup_word(text.strip()) == False):
            continue
        if(lang == "zh"):
            text_to_speech("你好,我是小度！")
            if(lang_cur == "ja"):
                nlp = spacy.load("zh_core_web_md")
                lang_cur = "zh"

        elif lang == "ja":
            text_to_speech("こんにちは，ミクロボです")
            if(lang_cur == "zh"):
                nlp = spacy.load("ja_core_news_md")
                lang_cur = "ja"
        else:
            text_to_speech("Hello")

        text_speech = recognize_speech(time_out = 10)
        print("Listening for speech...")
        print(f"Speech: {text_speech}")
        if(text_speech == ""):
            continue

        #会話モード/制御モードの切り替え
        if(lang_cur == "ja"):
            if "会話モード" in text_speech:
                mode = "chat"
                response = "会話モードに入りました"
                text_to_speech(response)
                continue
            elif "制御モード" in text_speech:
                mode = "control"
                response = "制御モードに入りました"
                text_to_speech(response)
                continue
        if(lang_cur == "zh"):
            if ("会话模式" in text_speech) or ("对话模式" in text_speech) or ("聊天模式" in text_speech):
                mode = "chat"
                response = "已经进入聊天模式"
                text_to_speech(response)
                continue
            elif "控制模式" in text_speech:
                mode = "control"
                response = "已经进入控制模式"
                text_to_speech(response)
                continue

        if mode == "control":
            command = parse_command(text_speech)
            print(f"解析结果: {command}")
            execute_command(command, lang_cur)
        elif mode == "chat":
            response = chat_with_gpt(text_speech)
            text_to_speech(response)

        rate.sleep()  # 等待下一条语音输入

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
