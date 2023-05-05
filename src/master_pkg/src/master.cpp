#include "ros/ros.h"

int main(int argc, char **argv) {
	ros::init(argc, argv, "master");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("joy", 1000, masterCallback);
	ros::spin();
	return 0;
}
