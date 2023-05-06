#include "ros/ros.h"
#include "sensor_msgs/Joy.h"

bool manual = false;
ros::Publisher cmd_vel_pub;

void joy_callback(const sensor_msgs::Joy::ConstPtr& joy_msg) {
	if (joy_msg->buttons[1]) {
		manual = true;
	} else if (joy_msg->buttons[2]) { // Enable automated mode
		manual = false;
	}
	if (manual) cmd_vel_pub.publish(joy_msg);
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "master");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("joy", 1000, joy_callback);
	cmd_vel_pub = n.advertise<sensor_msgs::Joy>("master/joy", 1);
	ros::spin();
	return 0;
}
