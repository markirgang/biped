import cv2
import numpy as np
import base64
import json
import os
from google import genai
from google.genai import types

class GeminiBipedBridge:
    def __init__(self):
        self.system_instruction = (
            "You are the friendly, intelligent AI brain of a 17-degree-of-freedom biped humanoid robot equipped with an eMeet Pixy camera and microphone. "
            "Your job is to:\n"
            "1. Produce warm, friendly, concise, conversational responses and ask the user questions about what they want the biped to do.\n"
            "2. Describe what you see in front of you clearly and concisely when presented with camera images.\n"
            "3. Control the biped robot using function calls whenever the user asks the biped to move, walk, strike a pose, wave, or stop.\n"
            "Always keep spoken chat responses natural and concise (1-3 sentences)."
        )

    def detect_hand_gestures(self, frame):
        """
        Analyzes camera frame for hand gestures (skin color segmentation + contour convexity defects)
        Returns: (gesture_name, finger_count, annotated_frame)
        """
        try:
            # Resize frame for processing speed
            resized = cv2.resize(frame, (320, 240))
            blur = cv2.GaussianBlur(resized, (5, 5), 0)
            hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)

            # Skin color range in HSV
            lower_skin = np.array([0, 20, 70], dtype=np.uint8)
            upper_skin = np.array([20, 255, 255], dtype=np.uint8)
            mask = cv2.inRange(hsv, lower_skin, upper_skin)

            # Morphological operations to remove noise
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
            mask = cv2.erode(mask, kernel, iterations=1)
            mask = cv2.dilate(mask, kernel, iterations=2)

            contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            if not contours:
                return "None", 0, frame

            # Find largest contour (assume it's hand)
            max_contour = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(max_contour)

            if area < 3000: # Filter out small noise
                return "None", 0, frame

            hull = cv2.convexHull(max_contour, returnPoints=False)
            if hull is None or len(hull) < 3:
                return "None", 0, frame

            defects = cv2.convexityDefects(max_contour, hull)
            finger_count = 0

            if defects is not None:
                for i in range(defects.shape[0]):
                    s, e, f, d = defects[i, 0]
                    start = tuple(max_contour[s][0])
                    end = tuple(max_contour[e][0])
                    far = tuple(max_contour[f][0])

                    # Calculate triangle sides to estimate defect angle
                    a = np.sqrt((end[0] - start[0])**2 + (end[1] - start[1])**2)
                    b = np.sqrt((far[0] - start[0])**2 + (far[1] - start[1])**2)
                    c = np.sqrt((end[0] - far[0])**2 + (end[1] - far[1])**2)

                    angle = np.arccos((b**2 + c**2 - a**2) / (2 * b * c + 1e-5))

                    # Angle less than 90 deg indicates gap between fingers
                    if angle <= np.pi / 2 and d > 1200:
                        finger_count += 1

                # Finger count is defects + 1
                finger_count += 1

            # Map finger count to gesture commands
            gesture = "None"
            if finger_count >= 4:
                gesture = "5_FINGERS_FORWARD"
            elif finger_count == 2 or finger_count == 3:
                gesture = "2_FINGERS_WAVE"
            elif finger_count == 1:
                gesture = "1_FINGER_ATTACK"

            return gesture, finger_count, frame
        except Exception as e:
            return "None", 0, frame

    def generate_chat_response(self, api_key, user_text="", image_bytes=None):
        """
        Sends text prompt and optional webcam snapshot to Google Gemini AI API.
        Returns: { text: str, action: dict, gesture: str }
        """
        if not api_key:
            return {
                "text": "Please provide a valid Google Gemini API Key in the settings tab to enable AI vision & voice features.",
                "action": None
            }

        client = genai.Client(api_key=api_key)

        # Declare Biped Robot Tools
        tools = [
            types.Tool(function_declarations=[
                types.FunctionDeclaration(
                    name="biped_walk",
                    description="Makes the biped walk or run in a direction.",
                    parameters=types.Schema(
                        type=types.Type.OBJECT,
                        properties={
                            "direction": types.Schema(
                                type=types.Type.STRING,
                                enum=["forward", "backward", "run", "run_forward", "left", "right", "turn_left", "turn_right"]
                            ),
                            "duration_seconds": types.Schema(type=types.Type.NUMBER)
                        },
                        required=["direction"]
                    )
                ),
                types.FunctionDeclaration(
                    name="biped_pose",
                    description="Sets biped stance or triggers a preset motion pose or dance routine.",
                    parameters=types.Schema(
                        type=types.Type.OBJECT,
                        properties={
                            "pose_name": types.Schema(
                                type=types.Type.STRING,
                                enum=[
                                    "Stand", "T-Pose", "Wave Right Hand", "Wave Left Hand", "Both Arms Up", "Dance", 
                                    "Squat", "Bow / Respect", "Sit / Park", "Sleep / Park", "Curious Tilt", 
                                    "Greeting Wave", "Low Stance", "High Stance", "Attack / Guard", "Walk in Place"
                                ]
                            )
                        },
                        required=["pose_name"]
                    )
                ),
                types.FunctionDeclaration(
                    name="biped_stop",
                    description="Stops all biped movement immediately.",
                    parameters=types.Schema(type=types.Type.OBJECT, properties={})
                )
            ])
        ]

        contents = []

        if image_bytes:
            contents.append(types.Part.from_bytes(
                data=image_bytes,
                mime_type="image/jpeg"
            ))

        prompt_text = user_text if user_text else "Describe what you see through your camera and ask me what you should do next!"
        contents.append(prompt_text)

        config = types.GenerateContentConfig(
            system_instruction=self.system_instruction,
            tools=tools,
            temperature=0.7
        )

        try:
            response = client.models.generate_content(
                model="gemini-3.8-flash",
                contents=contents,
                config=config
            )

            action = None
            text_response = response.text or ""

            # Check if Gemini executed function calls
            if response.function_calls:
                call = response.function_calls[0]
                action = {
                    "name": call.name,
                    "args": call.args
                }
                if not text_response:
                    text_response = f"Executing command: {call.name}"

            return {
                "text": text_response,
                "action": action
            }

        except Exception as e:
            return {
                "text": f"Gemini API Error: {str(e)}",
                "action": None
            }

    def generate_voice_chat_response(self, api_key, audio_bytes, mime_type="audio/webm", image_bytes=None):
        """
        Sends recorded voice audio directly to Gemini 3.8 Flash.
        Gemini transcribes the speech, answers conversationally, and executes robot tools.
        """
        if not api_key:
            return {"text": "Please provide a valid Google Gemini API Key.", "action": None}

        client = genai.Client(api_key=api_key)

        tools = [
            types.Tool(function_declarations=[
                types.FunctionDeclaration(
                    name="biped_walk",
                    description="Makes the biped walk or run in a direction.",
                    parameters=types.Schema(
                        type=types.Type.OBJECT,
                        properties={
                            "direction": types.Schema(
                                type=types.Type.STRING,
                                enum=["forward", "backward", "run", "run_forward", "left", "right", "turn_left", "turn_right"]
                            )
                        },
                        required=["direction"]
                    )
                ),
                types.FunctionDeclaration(
                    name="biped_pose",
                    description="Sets biped stance or triggers a preset motion pose or dance routine.",
                    parameters=types.Schema(
                        type=types.Type.OBJECT,
                        properties={
                            "pose_name": types.Schema(
                                type=types.Type.STRING,
                                enum=[
                                    "Stand", "T-Pose", "Wave Right Hand", "Wave Left Hand", "Both Arms Up", "Dance", 
                                    "Squat", "Bow / Respect", "Sit / Park", "Sleep / Park", "Curious Tilt", 
                                    "Greeting Wave", "Low Stance", "High Stance", "Attack / Guard", "Walk in Place"
                                ]
                            )
                        },
                        required=["pose_name"]
                    )
                ),
                types.FunctionDeclaration(
                    name="biped_stop",
                    description="Stops all biped movement immediately.",
                    parameters=types.Schema(type=types.Type.OBJECT, properties={})
                )
            ])
        ]

        contents = []

        if audio_bytes:
            contents.append(types.Part.from_bytes(
                data=audio_bytes,
                mime_type=mime_type
            ))

        if image_bytes:
            contents.append(types.Part.from_bytes(
                data=image_bytes,
                mime_type="image/jpeg"
            ))

        contents.append(
            "Listen to my spoken voice input in the audio file above.\n"
            "1. Transcribe the exact words I spoke.\n"
            "2. Respond to me concisely in 1-2 friendly, conversational sentences.\n"
            "3. If I asked the biped robot to move, walk, turn, wave, or pose, call the matching tool function.\n\n"
            "Format your text response as:\n"
            "TRANSCRIPTION: [exact spoken words]\n"
            "RESPONSE: [your conversational answer]"
        )

        config = types.GenerateContentConfig(
            system_instruction=self.system_instruction,
            tools=tools,
            temperature=0.7
        )

        try:
            response = client.models.generate_content(
                model="gemini-3.8-flash",
                contents=contents,
                config=config
            )

            action = None
            raw_text = response.text or ""
            user_transcription = ""
            text_response = raw_text

            if "TRANSCRIPTION:" in raw_text:
                parts = raw_text.split("RESPONSE:", 1)
                user_transcription = parts[0].replace("TRANSCRIPTION:", "").strip()
                if len(parts) > 1:
                    text_response = parts[1].strip()

            if response.function_calls:
                call = response.function_calls[0]
                action = {"name": call.name, "args": call.args}
                if not text_response:
                    text_response = f"Executing command: {call.name}"

            return {
                "transcription": user_transcription,
                "text": text_response,
                "action": action
            }

        except Exception as e:
            return {"text": f"Gemini Voice Processing Error: {str(e)}", "action": None}
