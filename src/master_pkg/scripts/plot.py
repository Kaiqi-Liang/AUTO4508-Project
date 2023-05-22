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
from sensor_msgs.msg import Image
from sensor_msgs.msg import NavSatFix

def gps_callback(data):
    df = pd.read_csv('../AUTO4508-Project/src/master_pkg/src/coordinate.csv', header=None)
    latitude = df.iloc[:,0].tolist()
    longitude = df.iloc[:,1].tolist()

    if not math.isnan(data.latitude):
        latitude.append(data.latitude)
    if not math.isnan(data.longitude):
        longitude.append(data.longitude)

    # Create a map projection
    projection = ccrs.PlateCarree()

    # Create a figure and axes with the projection
    _, ax = plt.subplots(subplot_kw={'projection': projection})

    # Plot the GPS coordinates
    ax.plot(longitude, latitude, 'bo', transform=projection)
    ax.set_title('GPS Coordinates Plot')

    fig = plt.gcf()
    fig.canvas.draw()
    img = np.array(fig.canvas.renderer._renderer)
    plt.close(1)

    bridge = CvBridge()
    image_msg = bridge.cv2_to_imgmsg(img, encoding="rgba8")

    pub = rospy.Publisher('plot_gps', Image, queue_size=10)
    pub.publish(image_msg)

if __name__ == '__main__':
    rospy.init_node('plot', anonymous=True)
    rospy.Subscriber('fix', NavSatFix, gps_callback)
    rospy.spin()
