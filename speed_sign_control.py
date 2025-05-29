#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO

class SpeedSignController:
    def __init__(self):
        rospy.init_node('speed_sign_controller', anonymous=True)
        self.bridge = CvBridge()
        self.model = YOLO("./tocdo_best.pt")
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.image_callback)
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        self.current_speed = 0.1
        self.rate = rospy.Rate(10)
        self.control_loop()

    def image_callback(self, msg):
        # Convert ROS image to OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Xử lý hình ảnh đơn giản
        processed_image = self.preprocess_image(cv_image)

        # Nhận diện biển báo
        results = self.model(processed_image)

        detected_speed = None
        for r in results:
            for box in r.boxes:
                conf = float(box.conf)
                if conf < 0.7:
                    continue  # Bỏ qua nếu độ chính xác thấp

                cls = r.names[int(box.cls)]

                if "20" in cls:
                    detected_speed = 0.2
                    print("DDDDDDDDDDDDDetected: 20 (conf: {:.2f})".format(conf))
                elif "40" in cls:
                    detected_speed = 0.4
                    print("DDDDDDDDDDDDDetected: 40 (conf: {:.2f})".format(conf))
                elif "60" in cls:
                    detected_speed = 0.6
                    print("DDDDDDDDDDDDDetected: 60 (conf: {:.2f})".format(conf))
                elif "80" in cls:
                    detected_speed = 0.8
                    print("DDDDDDDDDDDDDetected: 80 (conf: {:.2f})".format(conf))

        if detected_speed is not None:
            self.current_speed = detected_speed

    def preprocess_image(self, img):
        # Resize ảnh nhỏ lại để tăng tốc độ xử lý
        resized = cv2.resize(img, (640, 480))

        # Làm mịn ảnh để loại bỏ nhiễu nhẹ
        blurred = cv2.GaussianBlur(resized, (5, 5), 0)

        # Có thể thêm chuyển sang grayscale nếu model chấp nhận
        # gray = cv2.cvtColor(blurred, cv2.COLOR_BGR2GRAY)

        return blurred

    def control_loop(self):
        while not rospy.is_shutdown():
            twist = Twist()
            twist.linear.x = self.current_speed
            self.cmd_pub.publish(twist)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        SpeedSignController()
    except rospy.ROSInterruptException:
        pass
