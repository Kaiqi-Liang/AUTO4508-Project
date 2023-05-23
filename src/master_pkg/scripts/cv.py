#!/usr/bin/env python
import numpy as np
import cv2
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool

bucket = False

def get_contours_for_colour(img, lower: tuple[int, int, int], upper: tuple[int, int, int]):
    mask = cv2.inRange(img, np.array(lower), np.array(upper))
    contours = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    return contours[0] if len(contours) == 2 else contours[1]

def bucket_callback(data):
    if data.data:
        global bucket
        bucket = True

def image_callback(data):
    global bucket
    if not bucket:
        return

    img = np.reshape(np.frombuffer(data.data, dtype=np.uint8), (300, 300, 3))
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    contours = [
        *get_contours_for_colour(hsv, (160, 100, 50), (180, 255, 255)),
        *get_contours_for_colour(hsv, (20, 120, 70), (30, 255, 255)),
        *get_contours_for_colour(hsv, (90, 50, 50), (130, 255, 255)),
        *get_contours_for_colour(hsv, (45, 50, 50), (75, 255, 255)),
    ]

    objects = []
    for contour in contours:
        x, y, w, h = cv2.boundingRect(contour)
        if 25 < w < 100 and 30 < h < 100:
            objects.append(contour)
            if cv2.contourArea(contour) / (w * h) > 0.7:
                # If the area of the contour is at least 70% of its bounding box
                cv2.rectangle(img, (x, y), (x + w, y + h), (255, 0, 0), 1)
    cv2.drawContours(img, objects, -1, (0, 0, 255), 1)

    data.data = img.flatten().tobytes()
    pub = rospy.Publisher('labeled_image', Image, queue_size=10)
    pub.publish(data)
    bucket = False

if __name__ == '__main__':
    rospy.init_node('cv', anonymous=True)
    rospy.Subscriber('mobilenet_publisher/color/image', Image, image_callback)
    rospy.Subscriber('bucket', Bool, bucket_callback)
    rospy.spin()
