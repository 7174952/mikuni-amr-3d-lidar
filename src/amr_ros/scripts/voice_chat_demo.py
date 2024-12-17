#!/usr/bin/env python3
try:
    import torch
    import whisper
except ModuleNotFoundError as e:
    print(f"Missing module: {e.name}. Please ensure the required module is installed.")
    exit(1)

import spacy
import rospy
from std_msgs.msg import String
from zhconv import convert
import openai
import time
import os
import speech_recognition as sr
import pyaudio
from playsound import playsound
from typing import Dict, List

# 检查是否支持CUDA
if torch.cuda.is_available():
    device = "cuda"
    rospy.loginfo("Using CUDA (GPU)")
else:
    device = "cpu"
    rospy.loginfo("Using CPU")

# 初始化Whisper模型
model = whisper.load_model("base", device=device)

# 初始化spaCy
# nlp = spacy.load("zh_core_web_md")
nlp = spacy.load("ja_core_news_md")
lang_cur = "ja"

# ROS话题名称
voice_cmd_pub_topic = "/voice_ctrl_cmd"
# 初始化ROS发布者
robot_cmd_publisher = rospy.Publisher(voice_cmd_pub_topic, String, queue_size=10)

# 新增订阅主题
robot_cmd_sub_topic = "/voice_set_mode"
voice_mode = "chat"  # Initialize voice mode variable
is_continue_chat = False

def update_voice_mode(msg):
    global voice_mode
    voice_mode = msg.data
    rospy.loginfo(f"Updated voice_mode: {voice_mode}")

robot_cmd_subscriber = rospy.Subscriber(robot_cmd_sub_topic, String, callback=update_voice_mode)

# 从环境变量中读取OpenAI API密钥
openai.api_key = os.getenv('OPENAI_API_KEY')

# 初始化麦克风识别器
recognizer = sr.Recognizer()

audo_path = "/home/mikuni/catkin_ws/src/amr_ros/resource/"
#定义音频响应数据字典
voice_resp_ja = {
    "resp_wakeup": audo_path + "resp_wakeup_ja.mp3",
    "resp_start_to_run": audo_path + "resp_start_to_run_ja.mp3",
    "resp_stop_and_wait": audo_path + "resp_stop_and_wait_ja.mp3",
    "resp_unknown_cmd": audo_path + "resp_unknown_cmd_ja.mp3"
}
voice_resp_zh = {
    "resp_wakeup": audo_path + "resp_wakeup_zh.mp3",
    "resp_start_to_run": audo_path + "resp_start_to_run_zh.mp3",
    "resp_stop_and_wait": audo_path + "resp_stop_and_wait_zh.mp3",
    "resp_unknown_cmd": audo_path + "resp_unknown_cmd_zh.mp3"
}

def recognize_speech():
    """
    使用Whisper模型进行语音识别
    """
    global is_continue_chat
    global voice_mode

    start_time = time.time()

    print("请说话...")
    with sr.Microphone() as source:
        recognizer.adjust_for_ambient_noise(source, duration=1)  # 动态调整能量阈值

        while True:
            try:
                audio = recognizer.listen(source, timeout=5) # 添加超时机制
            except sr.WaitTimeoutError:
                print("Timeout waiting for speech input.")
                continue
            except sr.UnknownValueError:
                print("Could not understand audio.")
                continue

            with open("audio_clip.wav", "wb") as f:
                f.write(audio.get_wav_data())

            # 将音频保存为临时文件
            result = model.transcribe("audio_clip.wav")
            if(len(result['segments']) == 0):
                print("No transcribable segments found in audio. Retrying...")
                continue

            if ((result['text'].strip() == 'Thank you.') and (result['language'] == 'en') and (result['segments'][len(result['segments']) - 1]['end'] == 2.0)):
                print("Filtered out short placeholder audio.")
                continue

            if((result['language'] == 'en') or (result['language'] == 'zh') or  (result['language'] == 'ja')):
                ret_str = result['text']
                if(result['language'] == 'zh'):
                    ret_str = convert(result['text'], 'zh-hans')
            else:
                ret_str = ""
            if(int((time.time() - start_time) * 1000) > 15000):
                if(voice_mode == "chat"):
                    print("Chat Mode: back to wait wakeup word.")
                    is_continue_chat = False

            return ret_str, result['language']


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
    start_robot = ["开始","前进","出发","走っ","はしっ","進ん","進め","出発","しゅっぱつ","すすん","すすめ","スタット"]
    stop_robot = ["停车","停止","停","ストップ","止まっ","とまっ","止まれ","とまれ"]
    is_cmd_recognize = False;
    resp_cmd = ""
    if(lang == 'ja'):
        if command['action'] in  start_robot:
            response = voice_resp_ja["resp_start_to_run"]
            #publish
            resp_cmd = "start"
            robot_cmd_publisher.publish(resp_cmd)
            is_cmd_recognize = True
        elif command['action'] in stop_robot:
            response = voice_resp_ja["resp_stop_and_wait"]
            #publish
            resp_cmd = "stop"
            robot_cmd_publisher.publish(resp_cmd)
            is_cmd_recognize = True
        # else:
        #     response = "コマンドは分かりません"
        #     response = voice_resp_ja["resp_unknown_cmd"]

    if(lang == 'zh'):
        if command['action'] in  start_robot:
            response = voice_resp_zh["resp_start_to_run"]
            #publish
            resp_cmd = "start"
            robot_cmd_publisher.publish(resp_cmd)
            is_cmd_recognize = True
        elif (command['action'] in stop_robot) or (command['object'] in stop_robot):
            response = voice_resp_zh["resp_stop_and_wait"]
            #publish
            resp_cmd = "stop"
            robot_cmd_publisher.publish(resp_cmd)
            is_cmd_recognize = True
        # else:
            # response = voice_resp_zh["resp_unknown_cmd"]

    # 使用TTS播放执行结果
    try:
        if(is_cmd_recognize):
            playsound(response)
        else:
            print(f"Command not recognized: {command['action']}")
    except Exception as e:
        rospy.logwarn(f"Error playing sound: {e}")

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
        return "Sorry, I could not process your request. Please try again later."

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
        "ja": ["みくろぼ","ミクロボ","ミクロさん", "みくろさん", "ミクロサン","ミクロサウン"],
    }

    if (text.strip() in wakeup_dict["ja"]) or (text.strip() in wakeup_dict["zh"]):
        return True

    return False

def main():

    rospy.init_node("speech_recognition_node", anonymous=True)
    rate = rospy.Rate(10)  # 10Hz
    rospy.loginfo("Speech Recognition Node Started.")

    global voice_mode
    global nlp
    global lang_cur
    global is_continue_chat

    while not rospy.is_shutdown():
        print("Voice Mode:Chat. Listening for wakeup word:")
        text, lang = recognize_speech()
        print(f"Recognized: {text}")
        if(text == ""):
            continue

        if(voice_mode == "chat"):
            if(is_continue_chat == False):
                #check wakeup word
                if(find_wakeup_word(text.strip()) == False):
                    continue
                #setup language ja/zh
                if(lang == "zh"):
                    playsound(voice_resp_zh["resp_wakeup"])
                    if(lang_cur == "ja"):
                        nlp = spacy.load("zh_core_web_md")
                        lang_cur = "zh"
                elif lang == "ja":
                    playsound(voice_resp_ja["resp_wakeup"])
                    if(lang_cur == "zh"):
                        nlp = spacy.load("ja_core_news_md")
                        lang_cur = "ja"
                else:
                    text_to_speech("Hello, I'm guide robot")
                is_continue_chat = True #开始连续对话
                continue

            #online to get response from chatgpt
            response = chat_with_gpt(text)
            text_to_speech(response)

        elif(voice_mode=="control"):
            #setup language ja/zh
            if(lang == "zh"):
                if(lang_cur == "ja"):
                    nlp = spacy.load("zh_core_web_md")
                    lang_cur = "zh"
            elif lang == "ja":
                if(lang_cur == "zh"):
                    nlp = spacy.load("ja_core_news_md")
                    lang_cur = "ja"

            command = parse_command(text)
            print(f"解析结果: {command}")
            execute_command(command, lang_cur)
        #else: voice_mode=="off"
        #Do nothing

        rate.sleep()  # 等待下一条语音输入

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass

