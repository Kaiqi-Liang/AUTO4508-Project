#include "geometry_msgs/Twist.h"
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/Joy.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/NavSatFix.h"
#include <tf/tf.h>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

struct Cartesian {
	double x;
	double y;
	double z;
};

struct Coordinate {
	double latitude;
	double longitude;
	Coordinate(double latitude, double longitude)
	: latitude{latitude}
	, longitude{longitude} {}
};

bool manual = true;
bool facing_obstacle = false;
bool reached_waypoint = false;
bool found_bucket = false;
std::size_t waypoint_counter = 0;
std::size_t obstacle_timer = 0;
double current_heading = 0;
ros::Publisher joy_pub;
ros::Publisher cmd_vel_pub;
std::vector<Coordinate> coordinates;

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
	double e = std::sqrt((std::pow(a, 2) - std::pow(a * (1 - f), 2))
	                     / std::pow(a, 2)); // eccentricity
	double height = 5;
	double normal_curature_radius =
	   a / std::sqrt(1 - std::pow(e, 2) * std::pow(std::sin(phi), 2));
	return {
	   (normal_curature_radius + height) * std::cos(phi) * std::cos(lambda),
	   (normal_curature_radius + height) * std::cos(phi) * std::sin(lambda),
	   (normal_curature_radius * (1 - std::pow(e, 2)) + height) * std::sin(phi),
	};
}

void gps_callback(const sensor_msgs::NavSatFix::ConstPtr& gps_fix_msg) {
	if (manual or reached_waypoint) return;
	double angular_speed = 0;
	Cartesian goal = ellip2cart(coordinates[waypoint_counter].latitude,
	                            coordinates[waypoint_counter].longitude);
	Cartesian robot = ellip2cart(gps_fix_msg->latitude, gps_fix_msg->longitude);
	double distance =
	   std::sqrt(std::pow(robot.x - goal.x, 2) + std::pow(robot.y - goal.y, 2)
	             + std::pow(robot.z - goal.z, 2));
	double heading2goal = std::atan2(goal.y - robot.y, goal.x - robot.x);
	double angle = heading2goal - current_heading + M_PI / 2;
	if (std::abs(angle) > 0.3) { angular_speed = angle > 0 ? 0.3 : -0.3; }
	ROS_INFO("distance = %lf angle = %lf facing_obstacle = %d",
	         distance,
	         angle,
	         facing_obstacle);

	geometry_msgs::Twist cmd_vel_msg;
	if (distance > 1 and not facing_obstacle) {
		cmd_vel_msg.linear.x = 0.5;
		cmd_vel_msg.linear.y = 0;
		cmd_vel_msg.linear.z = 0;
		cmd_vel_msg.angular.x = 0;
		cmd_vel_msg.angular.y = 0;
		cmd_vel_msg.angular.z = angular_speed;
		cmd_vel_pub.publish(cmd_vel_msg);
	} else if (distance > 1) {
		// start a timer
		if (facing_obstacle) {
			++obstacle_timer;
			if (obstacle_timer > 10) { // distbug
				obstacle_timer = 0;
			}
		} else {
			obstacle_timer = 0;
		}
	} else {
		reached_waypoint = true;
		++waypoint_counter;
	}
}

void lidar_callback(const sensor_msgs::LaserScan::ConstPtr& lidar_scan_msg) {
	facing_obstacle = lidar_scan_msg->ranges[405] > 0
	                  and lidar_scan_msg->ranges[405] < 2;
	if (reached_waypoint) {
		double min_distance = DBL_MAX;
		double bucket_lidar_index;
		for (std::size_t i = 0; i < 812; ++i) {
			if (i < 385 and i > 425) {
				if (min_distance > lidar_scan_msg->ranges[i]) {
					min_distance = lidar_scan_msg->ranges[i];
					bucket_lidar_index = i;
				}
			}
		}
		// finish with this waypoint
		reached_waypoint = false;
	}
}

// void cv_callback(auto const& cv_msg) {
// 	if (reached_waypoint) found_bucket = true;
// }

void imu_callback(const sensor_msgs::Imu::ConstPtr& imu_msg) {
	tf::Quaternion q(imu_msg->orientation.x,
	                 imu_msg->orientation.y,
	                 imu_msg->orientation.z,
	                 imu_msg->orientation.w);
	tf::Matrix3x3 m(q);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);
	current_heading = yaw;
	// ROS_INFO("%lf %lf %lf", roll, pitch, yaw);
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "master");
	ros::NodeHandle n;

	std::ifstream coordinate("../AUTO4508-Project/src/master_pkg/src/coordinate.csv");
	std::vector<Coordinate> coordinates;
	std::string line;
	if (not coordinate.is_open()) {
		return 1;
	}
	while (std::getline(coordinate, line)) {
		std::stringstream ss{line};
		std::string token;
		std::getline(ss, token, ',');
		double latitude = std::stod(token);
		double longitude;
		ss >> longitude;
		coordinates.emplace_back(latitude, longitude);
	}
	coordinate.close();
	ROS_INFO("%lf %lf",
	         coordinates[waypoint_counter].latitude,
	         coordinates[waypoint_counter].longitude);

	ros::Subscriber joy_sub = n.subscribe("joy", 1000, joy_callback);
	ros::Subscriber gps_sub = n.subscribe("fix", 1000, gps_callback);
	ros::Subscriber imu_sub = n.subscribe("imu/data", 1000, imu_callback);
	// ros::Subscriber cv_sub = n.subscribe("found_bucket", 1000, cv_callback);
	ros::Subscriber lidar_sub =
	   n.subscribe("sick_tim_7xx/scan", 1000, lidar_callback);

	joy_pub = n.advertise<sensor_msgs::Joy>("master/joy", 1);
	cmd_vel_pub = n.advertise<geometry_msgs::Twist>("RosAria/cmd_vel", 1);

	ros::spin();
	return 0;
}
