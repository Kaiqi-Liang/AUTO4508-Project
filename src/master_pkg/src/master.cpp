#include "geometry_msgs/Twist.h"
#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/Image.h"

bool manual = true;
ros::Publisher joy_pub;
ros::Publisher cmd_vel_pub;

void joy_callback(const sensor_msgs::Joy::ConstPtr& joy_msg) {
	if (joy_msg->buttons[1]) {
		manual = true;
	} else if (joy_msg->buttons[2]) { // Enable automated mode
		manual = false;
	}
	if (manual) joy_pub.publish(joy_msg);
}

void gps_callback(const sensor_msgs::NavSatFix::ConstPtr& gps_fix_msg) {
	// ROS_INFO("%f %f", gps_fix_msg->latitude, gps_fix_msg->longitude);
	if (not manual) {
		geometry_msgs::Twist cmd_vel_msg;
		cmd_vel_msg.linear.x = 1;
		cmd_vel_msg.linear.y = 0;
		cmd_vel_msg.linear.z = 0;
		cmd_vel_msg.angular.x = 0;
		cmd_vel_msg.angular.y = 0;
		cmd_vel_msg.angular.z = 0;
		cmd_vel_pub.publish(cmd_vel_msg);
	}
}

void camera_callback(const sensor_msgs::Image::ConstPtr& camera_image) {
	// ROS_INFO("%d", camera_image->data[0]);
}

void lidar_callback(const sensor_msgs::LaserScan::ConstPtr& lidar_scan_msg) {
	// ROS_INFO("%f %f", lidar_scan_msg->intensities[0],
	// lidar_scan_msg->ranges[0]);
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "master");
	ros::NodeHandle n;
	ros::Subscriber joy_sub = n.subscribe("joy", 1000, joy_callback);
	ros::Subscriber gps_sub = n.subscribe("fix", 1000, gps_callback);
	ros::Subscriber camera_sub = n.subscribe("mobilenet_publisher/color/image", 1000, camera_callback);
	ros::Subscriber lidar_sub =
	   n.subscribe("sick_tim_7xx/scan", 1000, lidar_callback);
	joy_pub = n.advertise<sensor_msgs::Joy>("master/joy", 1);
	cmd_vel_pub = n.advertise<geometry_msgs::Twist>("RosAria/cmd_vel", 1);
	ros::spin();
	return 0;
}
