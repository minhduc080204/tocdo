from picamera2 import Picamera2
import time
import cv2
import RPi.GPIO as GPIO
from ultralytics import YOLO

class SpeedSignController:
    def __init__(self):
        # Setup camera
        self.picam2 = Picamera2()
        config = self.picam2.create_preview_configuration(main={"format": "RGB888", "size": (32, 32)})
        self.picam2.configure(config)
        self.picam2.start()
        time.sleep(2)

        # Load model YOLO
        self.model = YOLO("./tocdo_best.pt")

        # Setup GPIO cho L298N
        GPIO.setmode(GPIO.BCM)

        # Chn di?u khi?n motor tri
        self.ENA = 18
        self.IN1 = 23
        self.IN2 = 24

        # Chn di?u khi?n motor ph?i
        self.ENB = 13
        self.IN3 = 27
        self.IN4 = 22

        GPIO.setup(self.ENA, GPIO.OUT)
        GPIO.setup(self.IN1, GPIO.OUT)
        GPIO.setup(self.IN2, GPIO.OUT)
        GPIO.setup(self.ENB, GPIO.OUT)
        GPIO.setup(self.IN3, GPIO.OUT)
        GPIO.setup(self.IN4, GPIO.OUT)

        # PWM v?i t?n s? 1000 Hz
        self.pwmA = GPIO.PWM(self.ENA, 1000)
        self.pwmB = GPIO.PWM(self.ENB, 1000)
        self.pwmA.start(0)
        self.pwmB.start(0)

        self.current_speed = 0  # T?c d? ban d?u (duty cycle 0-100)

        self.control_loop()

    def capture_frame(self):
        return self.picam2.capture_array()

    def detect_speed(self, frame):
        processed = self.preprocess_image(frame)
        results = self.model(processed)

        detected_speed = None
        for r in results:
            for box in r.boxes:
                conf = float(box.conf)
                if conf < 0.7:
                    continue
                cls = r.names[int(box.cls)]

                if "20" in cls:
                    detected_speed = 20
                    print(f"Detected speed limit: 20 (conf: {conf:.2f})")
                elif "40" in cls:
                    detected_speed = 40
                    print(f"Detected speed limit: 40 (conf: {conf:.2f})")
                elif "60" in cls:
                    detected_speed = 60
                    print(f"Detected speed limit: 60 (conf: {conf:.2f})")
                elif "80" in cls:
                    detected_speed = 80
                    print(f"Detected speed limit: 80 (conf: {conf:.2f})")

        return detected_speed

    def preprocess_image(self, img):
        blurred = cv2.GaussianBlur(img, (5, 5), 0)
        return blurred

    def drive_forward(self, speed):
        # Set hu?ng ti?n cho motor tri
        GPIO.output(self.IN1, GPIO.HIGH)
        GPIO.output(self.IN2, GPIO.LOW)

        # Set hu?ng ti?n cho motor ph?i
        GPIO.output(self.IN3, GPIO.HIGH)
        GPIO.output(self.IN4, GPIO.LOW)

        # i?u ch?nh t?c d? (duty cycle 0-100)
        self.pwmA.ChangeDutyCycle(speed)
        self.pwmB.ChangeDutyCycle(speed)
        print(f"Set speed PWM: {speed}% - Motor running at this speed")
    def stop(self):
        self.pwmA.ChangeDutyCycle(0)
        self.pwmB.ChangeDutyCycle(0)
        GPIO.output(self.IN1, GPIO.LOW)
        GPIO.output(self.IN2, GPIO.LOW)
        GPIO.output(self.IN3, GPIO.LOW)
        GPIO.output(self.IN4, GPIO.LOW)
        print("Stopped motors")

    def map_speed_to_duty_cycle(self, speed_limit):
        # Chuy?n d?i t?c d? gi?i h?n (20, 40, 60, 80) thnh duty cycle (0-100)
        duty_cycle = (speed_limit / 80) * 100
        # ?m b?o duty cycle d? l?n d? m to quay (t?i thi?u 30%)
        duty_cycle = max(30, min(duty_cycle, 100))
        return duty_cycle

    def control_loop(self):
        try:
            while True:
                frame = self.capture_frame()
                detected_speed = self.detect_speed(frame)

                if detected_speed is not None:
                    # Ch? c?p nh?t t?c d? khi pht hi?n bi?n bo m?i
                    self.current_speed = self.map_speed_to_duty_cycle(detected_speed)
                    print(f"Updated speed to: {self.current_speed}% based on detected speed limit: {detected_speed}")

                # Ti?p t?c ch?y v?i t?c d? hi?n t?i
                if self.current_speed > 0:
                    self.drive_forward(self.current_speed)
                else:
                    self.stop()

                time.sleep(0.1)  # L?p m?i 0.1s

        except KeyboardInterrupt:
            self.stop()
            GPIO.cleanup()
            print("Program terminated")

if __name__ == '__main__':
    SpeedSignController()
