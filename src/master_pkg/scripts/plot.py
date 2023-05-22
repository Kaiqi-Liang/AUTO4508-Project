#!/usr/bin/env python
import math
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import cartopy.crs as ccrs
from io import BytesIO
import cv2
import rospy
from cv_bridge import CvBridge
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
from sensor_msgs.msg import NavSatFix

goal_latitudes = []
goal_longitudes = []
robot_latitudes = []
robot_longitudes = []
obstacle_latitudes = []
obstacle_longitudes = []
bucket_latitudes = []
bucket_longitudes = []

def obstacle_callback(gps):
    if gps.z == 1:
        bucket_latitudes.append(gps.x)
        bucket_longitudes.append(gps.y)
    else:
        obstacle_latitudes.append(gps.x)
        obstacle_longitudes.append(gps.y)

def gps_callback(gps):
    if not math.isnan(gps.latitude):
        robot_latitudes.append(gps.latitude)
    if not math.isnan(gps.longitude):
        robot_longitudes.append(gps.longitude)

    # Create a map projection
    projection = ccrs.PlateCarree()

    # Create a figure and axes with the projection
    _, ax = plt.subplots(subplot_kw={'projection': projection})

    # Plot the GPS coordinates
    ax.plot(goal_longitudes, goal_latitudes, 'mo', label='waypoint', transform=projection)
    if len(robot_latitudes) > 0 and len(robot_longitudes) > 0:
        ax.plot(robot_longitudes, robot_latitudes, 'yo', label='path', transform=projection)
    if len(obstacle_latitudes) > 0 and len(obstacle_longitudes) > 0:
        ax.plot(obstacle_longitudes, obstacle_latitudes, 'ko', label='obstacle', transform=projection)
    if len(bucket_latitudes) > 0 and len(bucket_longitudes) > 0:
        ax.plot(bucket_longitudes, bucket_latitudes, 'ro', label='bucket', transform=projection)
    ax.set_title('GPS Coordinates Plot')

    plt.legend()
    fig = plt.gcf()
    fig.canvas.draw()
    img = np.array(fig.canvas.renderer._renderer)
    plt.close(1)

    bridge = CvBridge()
    image_msg = bridge.cv2_to_imgmsg(img, encoding="rgba8")

    pub = rospy.Publisher('plot_gps', Image, queue_size=10)
    pub.publish(image_msg)

if __name__ == '__main__':
    df = pd.read_csv('../AUTO4508-Project/src/master_pkg/src/coordinate.csv', header=None)
    goal_latitudes = df.iloc[:,0].tolist()
    goal_longitudes = df.iloc[:,1].tolist()

    rospy.init_node('plot', anonymous=True)
    rospy.Subscriber('fix', NavSatFix, gps_callback)
    rospy.Subscriber('obstacle_gps', Point, obstacle_callback)
    rospy.spin()
