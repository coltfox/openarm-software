import queue, sys, numpy as np
import sounddevice as sd
from faster_whisper import WhisperModel
from ollama import chat
import pyttsx4
from pydantic import BaseModel
from typing import Literal


# ---------------- State Machine Configuration ---------------- #
class StateSelection(BaseModel):
    message: str
    state: Literal[
        'forward_kinematics',
        'inverse_kinematics',
        'object_detection',
        'grab_object',
        'rotate_left',
        'rotate_right'
    ]
    parameters: dict = {}
    confidence: float


# ---------------- Robotic Control Functions ---------------- #
def execute_state(state: StateSelection):
    """Execute the selected state on physical hardware"""
    print(f"Executing {state.state} with params {state.parameters}")
    # Add actual motor control logic here
    if state.state == 'rotate_left':
        pass  # Implement rotation logic
    elif state.state == 'grab_object':
        pass  # Implement gripper logic


# ---------------- Modified Prompt and Initialization ---------------- #
prompt = """You are a robotic arm that can contorl a state machine by returning outputs:
User Modes:
- forward_kinematics: Enters computer vision mode where user controls using fk, only enter this mode if user requests it
- inverse_kinematics: Enters cv mode where user controls using ik, only enter this mode if user requests it
Your tools: 
- object_detection: Locate and identify objects in workspace
- grab_object: Close gripper on detected object
- rotate_left/rotate_right: Adjust base angle

Other than that you are just a freindly helper

"""

messages = [{'role': 'system', 'content': prompt}]

# ---------------- Existing Audio Pipeline (Unmodified) ---------------- #
engine = pyttsx4.init()
model = WhisperModel("large", device="cuda" , num_workers=2)
audio_q = queue.Queue()


def audio_callback(indata, frames, time, status):
    if status: print(status, file=sys.stderr)
    audio_q.put(indata.copy())


with sd.InputStream(samplerate=16000, channels=1, dtype="int32",
                    callback=audio_callback, blocksize=int(16000 * 5)):
    print("🎙️  Say command (e.g. 'Rotate left 30 degrees', 'Grab the red block'):")
    buffer = np.empty((0,), dtype=np.int16)

    while True:
        data = audio_q.get()
        buffer = np.concatenate([buffer, data.flatten()])

        if len(buffer) >= 16000 * 5:
            segment = buffer.astype(np.float32) / 32768.0
            user_input, _ = model.transcribe(segment, language="en", beam_size=5, vad_filter=True)
            user_text = " ".join([seg.text.strip() for seg in user_input]).strip()[:800]

            if user_text:
                # Get structured state selection from LLM
                response = chat('deepseek-r1:1.5b',format=StateSelection.model_json_schema(),options={'temperature': .1}, messages=messages + [
                    {'role': 'user', 'content': user_text},
                ])

                # Voice confirmation
                engine.say(response.message.content)
                engine.runAndWait()

                try:
                    # Parse and validate response
                    state = StateSelection.model_validate_json(response.message.content)
                    execute_state(state)

                    # Voice confirmation
                    engine.say(f"Executing {state.state.replace('_', ' ')}")
                    engine.runAndWait()

                except Exception as e:
                    print(f"Invalid response: {e}")
                    engine.say("Please try a different command")
                    engine.runAndWait()

            buffer = np.empty((0,), dtype=np.int16)