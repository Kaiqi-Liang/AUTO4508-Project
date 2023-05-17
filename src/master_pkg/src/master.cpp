#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/NavSatFix.h"
#include "geometry_msgs/Twist.h"

struct Cartesian {
	double x;
	double y;
	double z;
};

bool manual = true;
bool facing_obstacle = false;
ros::Publisher joy_pub;
ros::Publisher cmd_vel_pub;
double latitude = -31.98020163112797;
double longitude = 115.8175287621561;
double current_heading = 0;

void joy_callback(const sensor_msgs::Joy::ConstPtr& joy_msg) {
	if (joy_msg->buttons[1]) {
		manual = true;
	} else if (joy_msg->buttons[2]) { // Enable automated mode
		manual = false;
	}
	if (manual) joy_pub.publish(joy_msg);
}

Cartesian ellip2cart(double phi, double lambda) {
	phi *= M_PI / 180;
	lambda *= M_PI / 180;
	double a = 6378137; // semi-major axis (WGS84) [m]
	double f = 1 / 298.257223563; // earth flattening (WGS84)
	double e = std::sqrt((std::pow(a, 2) - std::pow(a * (1 - f), 2)) / std::pow(a, 2)); // excentricity
	double height = 5;
	double normal_curature_radius = a / std::sqrt(1 - std::pow(e, 2) * std::pow(std::sin(phi), 2));
	return {
		(normal_curature_radius + height) * std::cos(phi) * std::cos(lambda),
		(normal_curature_radius + height) * std::cos(phi) * std::sin(lambda),
		(normal_curature_radius * (1 - std::pow(e, 2)) + height) * std::sin(phi),
	};
}

void gps_callback(const sensor_msgs::NavSatFix::ConstPtr& gps_fix_msg) {
	double angular_speed = 0;
	Cartesian goal = ellip2cart(latitude, longitude);
	Cartesian robot = ellip2cart(gps_fix_msg->latitude, gps_fix_msg->longitude);
	distance = std::sqrt(std::pow(robot.x - goal.x, 2) + std::pow(robot.y - goal.y, 2) + std::pow(robot.z - goal.z, 2));
	double heading2goal = std::atan2(goal.y - robot.y, goal.x - robot.x);
	double angle = heading2goal - current_heading;
	if (std::abs(angle) > 0.3) {
		angular_speed = angle > 0 ? 0.1 : -0.1;
	}
	ROS_INFO("distance = %lf angle = %lf manual = %d facing_obstacle = %d", distance, angle, manual, facing_obstacle);
	if (distance > 2 and not manual) {
		geometry_msgs::Twist cmd_vel_msg;
		cmd_vel_msg.linear.x = 1;
		cmd_vel_msg.linear.y = 0;
		cmd_vel_msg.linear.z = 0;
		cmd_vel_msg.angular.x = 0;
		cmd_vel_msg.angular.y = 0;
		cmd_vel_msg.angular.z = angular_speed;
		cmd_vel_pub.publish(cmd_vel_msg);
	}
}

void lidar_callback(const sensor_msgs::LaserScan::ConstPtr& lidar_scan_msg) {
	facing_obstacle = lidar_scan_msg->ranges[405] < 1.5;
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "master");
	ros::NodeHandle n;
	ros::Subscriber joy_sub = n.subscribe("joy", 1000, joy_callback);
	ros::Subscriber gps_sub = n.subscribe("fix", 1000, gps_callback);
	ros::Subscriber lidar_sub =
	   n.subscribe("sick_tim_7xx/scan", 1000, lidar_callback);
	joy_pub = n.advertise<sensor_msgs::Joy>("master/joy", 1);
	cmd_vel_pub = n.advertise<geometry_msgs::Twist>("RosAria/cmd_vel", 1);
	ros::spin();
	return 0;
}
