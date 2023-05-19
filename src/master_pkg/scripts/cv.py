#!/usr/bin/env python
import numpy as np
import cv2
import rospy
from sensor_msgs.msg import Image

def callback(data):
    img = np.reshape(np.frombuffer(data.data, dtype=np.uint8), (300, 300, 3))
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    lower_red = np.array([160, 100, 50])
    upper_red = np.array([180, 255, 255])
    mask = cv2.inRange(hsv, lower_red, upper_red)
    contours = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]
    bounding_boxes = []
    for contour in contours:
        x, y, w, h = cv2.boundingRect(contour)
        if w > 20 and h > 30:
            bounding_boxes.append(((x, y), (x + w, y + h)))
    # if len(bounding_box) > 0:
    #     pub = rospy.Publisher('found_bucket', bool, queue_size=10)
    #     pub.publish(True)
    for bounding_box in bounding_boxes:
        cv2.rectangle(img, *bounding_box, (255, 0, 0), 1)
    data.data = img.flatten().tobytes()
    pub = rospy.Publisher('labeled_image', Image, queue_size=10)
    pub.publish(data)

if __name__ == '__main__':
    rospy.init_node('cv', anonymous=True)
    rospy.Subscriber('mobilenet_publisher/color/image', Image, callback)
    rospy.spin()
