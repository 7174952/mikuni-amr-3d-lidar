#!/usr/bin/env python3
from openai import OpenAI
import os
from playsound import playsound

client = OpenAI()
client.api_key = os.getenv('OPENAI_API_KEY')


speech_file_path = "chat_introduce_lab_zh" + ".mp3"
content_text = "这里是实验室测试楼"

response = client.audio.speech.create(
    model="tts-1",
    voice="nova",
    input=content_text
)

response.stream_to_file(speech_file_path)

playsound(speech_file_path)
print("Done in:" + speech_file_path)
