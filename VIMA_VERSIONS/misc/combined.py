import cv2
import mediapipe as mp
import numpy as np
import serial
import time
import math
from threading import Thread


class CombinedSystem:
    def __init__(self):
        # Camera & CV setup
        self.cap = cv2.VideoCapture(0)
        self.mp_pose = mp.solutions.pose
        self.mp_hands = mp.solutions.hands
        self.pose = self.mp_pose.Pose(min_detection_confidence=0.5)
        self.hands = self.mp_hands.Hands(max_num_hands=1)

        # Serial setup
        self.ser = None
        self.serial_port = 'COM3'
        self.baud_rate = 115200
        self.connect_serial()

        # State management
        self.last_valid_angles = [0] * 6
        self.running = True
        self.landmark_history = []

        # Start processing thread
        Thread(target=self.process_frames, daemon=True).start()

    def connect_serial(self):
        try:
            self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=0.1)
            print("Serial connected")
        except Exception as e:
            print(f"Serial error: {e}")

    def robust_landmark_detection(self, results):
        if not results.pose_landmarks:
            if self.landmark_history:
                return self.landmark_history[-1]  # Use last valid
            return None
        return results.pose_landmarks

    def calculate_angles(self, landmarks):
        # Simplified inverse kinematics calculation
        angles = []
        key_joints = [11, 13, 15, 23, 25, 27]  # Shoulders/elbows/hips
        for j in key_joints:
            lm = landmarks.landmark[j]
            angles.append(self.normalize_angle(lm.x, lm.y, lm.z))
        return angles

    def normalize_angle(self, x, y, z):
        # Convert coordinates to robot's range [-90, 90]
        return 180 * (x - 0.5)  # Simplified mapping

    def smooth_angles(self, new_angles):
        # Apply EMA smoothing
        smoothed = [0.3 * new + 0.7 * old for new, old in zip(new_angles, self.last_valid_angles)]
        self.last_valid_angles = smoothed
        return smoothed

    def send_to_arduino(self, angles):
        if not self.ser:
            return

        cmd = ",".join(f"{a:.1f}" for a in angles) + "\n"
        try:
            self.ser.write(cmd.encode())
        except Exception as e:
            print(f"Serial write error: {e}")
            self.connect_serial()

    def process_frames(self):
        while self.running:
            success, frame = self.cap.read()
            if not success:
                continue

            # Process pose
            results = self.pose.process(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
            landmarks = self.robust_landmark_detection(results)

            if landmarks:
                # Calculate and smooth angles
                raw_angles = self.calculate_angles(landmarks)
                smooth_angles = self.smooth_angles(raw_angles)
                self.send_to_arduino(smooth_angles)

                # Update history
                self.landmark_history.append(landmarks)
                if len(self.landmark_history) > 5:
                    self.landmark_history.pop(0)

            # Show preview
            cv2.imshow('Pose Tracking', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.running = False

    def __del__(self):
        self.running = False
        self.cap.release()
        cv2.destroyAllWindows()
        if self.ser:
            self.ser.close()


if __name__ == "__main__":
    system = CombinedSystem()
    while system.running:
        time.sleep(0.1)