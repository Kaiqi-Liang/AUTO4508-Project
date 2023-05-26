#!/usr/bin/env python
import numpy as np
import cv2
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float64

distance: Float64 = None

def get_contours_for_colour(img, lower, upper):
    mask = cv2.inRange(img, np.array(lower), np.array(upper))
    contours = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    return contours[0] if len(contours) == 2 else contours[1]

def distance_callback(data):
    global distance
    distance = data.data

def image_callback(data):
    global distance
    if distance is None:
        return

    img = np.reshape(np.frombuffer(data.data, dtype=np.uint8), (300, 300, 3))
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    contours = [
        *get_contours_for_colour(hsv, (160, 100, 50), (180, 255, 255)), # red
        *get_contours_for_colour(hsv, (10, 120, 70), (40, 255, 255)), # yellow
    ]

    objects = []
    for contour in contours:
        x, y, w, h = cv2.boundingRect(contour)
        if 20 < w < 250 and 20 < h < 250:
            objects.append(contour)
            if cv2.contourArea(contour) / (w * h) > 0.7:
                # If the area of the contour is at least 70% of its bounding box
                cv2.rectangle(img, (x, y), (x + w, y + h), (255, 0, 0), 1)
    cv2.drawContours(img, objects, -1, (0, 0, 255), 1)

    # Define the text to be added
    text = f"Distance to cone: {round(distance, 2)}m"

    # Specify the font settings
    font = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = 0.7
    font_color = (0, 255, 0)  # Green color (BGR format)
    thickness = 1

    # Calculate the position for placing the text at the bottom-left corner
    text_x = 5
    text_y = img.shape[0] - 5

    # Add the text to the image
    cv2.putText(img, text, (text_x, text_y), font, font_scale, font_color, thickness)
    cv2.imwrite(f"../catkin_ws/{distance}.jpg", img)

    data.data = img.flatten().tobytes()
    pub = rospy.Publisher('labeled_image', Image, queue_size=10)
    pub.publish(data)
    distance = None

if __name__ == '__main__':
    rospy.init_node('cv', anonymous=True)
    rospy.Subscriber('mobilenet_publisher/color/image', Image, image_callback)
    rospy.Subscriber('distance', Float64, distance_callback)

    data = Image()
    pub = rospy.Publisher('labeled_image', Image, queue_size=10)
    data.data = cv2.imread("../AUTO4508-Project/default.jpg")
    pub.publish(data)

    rospy.spin()
