#!/usr/bin/env python3
import subprocess

def set_default_audio_device(device_name, device_type, inout_info):
    """
    Set default audio device using pactl.
    Args:
        device_name (str): Name of the audio device.
        device_type (str): 'source' for input, 'sink' for output.
    """
    # get devices list
    devices = subprocess.check_output(["pactl", "list", device_type + "s", "short"]).decode().splitlines()

    # search device ID by device_name
    device_id = None
    for line in devices:
        print(f"{device_type}:{line}")
        if (inout_info in line) and (device_name in line):
            device_id = line.split("\t")[0]
            break

    if device_id:
        # setup default device
        subprocess.run(["pactl", "set-default-" + device_type, device_id])
        print(f"Default {device_type} set to: {device_name}")
    else:
        print(f"Device '{device_name}' not found.")

# set default voice input device
set_default_audio_device("USB_Microphone", "source","alsa_input")

# update default voice output device
set_default_audio_device("USB_Sound", "sink", "alsa_output")
