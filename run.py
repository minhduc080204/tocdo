import cv2
from ultralytics import YOLO
import time
import RPi.GPIO as GPIO  # hoặc thư viện điều khiển motor khác

# Khởi tạo camera
cap = cv2.VideoCapture(0)

# Khởi tạo model YOLO
model = YOLO("/home/minhduc/catkin_ws/src/speed_sign_control/model/tocdo_best.pt")

# Khởi tạo GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(18, GPIO.OUT)  # ví dụ chân PWM cho motor

pwm = GPIO.PWM(18, 100)  # 100 Hz
pwm.start(0)

current_speed = 10  # ví dụ giá trị speed 0-100%

def set_speed(speed_percent):
    pwm.ChangeDutyCycle(speed_percent)

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # Resize + blur giống như bạn làm
        resized = cv2.resize(frame, (640, 480))
        blurred = cv2.GaussianBlur(resized, (5, 5), 0)

        results = model(blurred)

        detected_speed = None
        for r in results:
            for box in r.boxes:
                conf = float(box.conf)
                if conf < 0.7:
                    continue

                cls = r.names[int(box.cls)]
                if "20" in cls:
                    detected_speed = 20
                elif "40" in cls:
                    detected_speed = 40
                elif "60" in cls:
                    detected_speed = 60
                elif "80" in cls:
                    detected_speed = 80

        if detected_speed is not None:
            print(f"Detected speed sign: {detected_speed}")
            current_speed = detected_speed

        # Chuyển giá trị detected_speed thành PWM hoặc lệnh điều khiển
        set_speed(current_speed)

        time.sleep(0.1)

except KeyboardInterrupt:
    pass

finally:
    pwm.stop()
    GPIO.cleanup()
    cap.release()
