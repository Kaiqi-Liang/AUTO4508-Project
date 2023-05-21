#!/usr/bin/env python
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import cartopy.crs as ccrs
from io import BytesIO
import cv2
import rospy
from sensor_msgs.msg import Image
from sensor_msgs.msg import NavSatFix

def gps_callback(data):
    df = pd.read_csv('coordinate.csv', header=None)
    latitude = df.iloc[:,0].tolist()
    longitude = df.iloc[:,1].tolist()

    latitude.append(data.latitude)
    longitude.append(data.longitude)

    # Create a map projection
    projection = ccrs.PlateCarree()

    # Create a figure and axes with the projection
    _, ax = plt.subplots(subplot_kw={'projection': projection})

    # Plot the GPS coordinates
    ax.plot(longitude, latitude, 'bo', transform=projection)
    ax.set_title('GPS Coordinates Plot')

    # Show the plot
    buffer = BytesIO()
    plt.savefig(buffer, format='png')
    buffer.seek(0)

    img = np.asarray(bytearray(buffer.read()), dtype=np.uint8)
    data.data = img.tobytes()
    pub = rospy.Publisher('plot_gps', Image, queue_size=10)
    pub.publish(data)

if __name__ == '__main__':
    rospy.init_node('plot', anonymous=True)
    rospy.Subscriber('gps', NavSatFix, gps_callback)
    rospy.spin()
