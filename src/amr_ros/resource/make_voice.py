#!/usr/bin/env python3
from openai import OpenAI
import os
from playsound import playsound

client = OpenAI()
client.api_key = os.getenv('OPENAI_API_KEY')


speech_file_path = "resp_start_to_run_zh" + ".mp3"
content_text = "出发了"

response = client.audio.speech.create(
    model="tts-1",
    voice="nova",
    input=content_text
)

response.stream_to_file(speech_file_path)

playsound(speech_file_path)
print("Done in:" + speech_file_path)
