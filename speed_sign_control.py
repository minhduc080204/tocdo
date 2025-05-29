from picamera2 import Picamera2
import time
import cv2
from gpiozero import PWMOutputDevice
from ultralytics import YOLO

class SpeedSignController:
    def __init__(self):
        # Setup camera
        self.picam2 = Picamera2()
        config = self.picam2.create_preview_configuration(main={"format": "RGB888", "size": (640, 480)})
        self.picam2.configure(config)
        self.picam2.start()
        time.sleep(2)

        # Load YOLO model
        self.model = YOLO("./tocdo_best.pt")

        # Setup motor control using PWM
        self.motor = PWMOutputDevice(pin=18)  # GPIO18 (Physical pin 12), thay b?ng pin di?u khi?n th?c t?
        self.current_speed = 0.1

        # Main control loop
        self.control_loop()

    def capture_frame(self):
        return self.picam2.capture_array()

    def detect_speed(self, frame):
        processed = self.preprocess_image(frame)
        results = self.model(processed)
        for r in results:
            for box in r.boxes:
                conf = float(box.conf)
                if conf < 0.7:
                    continue

                cls = r.names[int(box.cls)]
                if "20" in cls:
                    print("Detected: 20 (conf: {:.2f})".format(conf))
                    return 0.2
                elif "40" in cls:
                    print("Detected: 40 (conf: {:.2f})".format(conf))
                    return 0.4
                elif "60" in cls:
                    print("Detected: 60 (conf: {:.2f})".format(conf))
                    return 0.6
                elif "80" in cls:
                    print("Detected: 80 (conf: {:.2f})".format(conf))
                    return 0.8
        return None

    def preprocess_image(self, img):
        resized = cv2.resize(img, (640, 480))
        blurred = cv2.GaussianBlur(resized, (5, 5), 0)
        return blurred

    def control_motor(self, speed):
        self.motor.value = min(max(speed, 0.0), 1.0)

    def control_loop(self):
        try:
            while True:
                frame = self.capture_frame()
                detected_speed = self.detect_speed(frame)
                if detected_speed is not None:
                    self.current_speed = detected_speed

                self.control_motor(self.current_speed)
                time.sleep(0.1)
        except KeyboardInterrupt:
            self.motor.value = 0
            print("Stopped")

if __name__ == '__main__':
    SpeedSignController()
