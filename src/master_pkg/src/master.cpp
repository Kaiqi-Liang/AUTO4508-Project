#include "geometry_msgs/Twist.h"
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/Joy.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/NavSatFix.h"
#include "std_msgs/Bool.h"
#include <tf/tf.h>
#include <unordered_map>
#include <cmath>
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

enum State {
	DRIVING, // Driving towards the next waypoint
	ROTATING, // Rotating away for the obstacle after waiting for them to move
	FOLLOWING, // Following the obstacle keeping it on the right
	DETECTING, // Looking for the bucket after reaching a waypoint
	TURNING, // Turning towards the bucket after detecting it
};
State state = DRIVING;

constexpr std::size_t LIDAR_FRONT = 405;
constexpr std::size_t LIDAR_FRONT_RIGHT = 395;
constexpr std::size_t LIDAR_FRONT_LEFT = 415;
bool manual = true;
bool facing_obstacle = false;
bool found_bucket = false;
std::size_t waypoint_counter = 0;
std::size_t obstacle_timer = 0;
double current_heading = 0;
ros::Publisher joy_pub;
ros::Publisher cmd_vel_pub;
ros::Publisher cv_pub;
std::vector<Coordinate> coordinates;

void joy_callback(const sensor_msgs::Joy::ConstPtr& joy_msg) {
	if (joy_msg->buttons[1]) {
		manual = true;
	} else if (joy_msg->buttons[2]) { // automated mode
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
	if (manual or std::isnan(gps_fix_msg->latitude)
	    or std::isnan(gps_fix_msg->longitude) or state != DRIVING)
		return;
	double angular_speed = 0;
	// ROS_INFO("counter=%ld, latitude=%lf, longitude=%lf",waypoint_counter,
	// coordinates[waypoint_counter].latitude,
	// coordinates[waypoint_counter].longitude);
	Cartesian goal = ellip2cart(coordinates[waypoint_counter].latitude,
	                            coordinates[waypoint_counter].longitude);
	Cartesian robot = ellip2cart(gps_fix_msg->latitude, gps_fix_msg->longitude);
	ROS_INFO("x=%lf, y=%lf, z=%lf", goal.x, goal.y, goal.z);
	ROS_INFO("x=%lf, y=%lf, z=%lf", robot.x, robot.y, robot.z);
	double distance =
	   std::sqrt(std::pow(robot.x - goal.x, 2) + std::pow(robot.y - goal.y, 2)
	             + std::pow(robot.z - goal.z, 2));
	double heading2goal = std::atan2(goal.x - robot.x, goal.y - robot.y);
	ROS_INFO("distance=%lf heading2goal=%lf", distance, heading2goal);
	double angle = heading2goal - current_heading + M_PI / 2;
	if (angle > M_PI) angle -= 2 * M_PI;
	if (std::abs(angle) > 0.1) angular_speed = angle > 0 ? 0.3 : -0.3;
	ROS_INFO("angle=%lf heading=%lf facing_obstacle=%d",
	         angle,
	         current_heading,
	         facing_obstacle);

	geometry_msgs::Twist cmd_vel_msg;
	if (distance > 1 and not facing_obstacle) {
		cmd_vel_msg.linear.x = 0.5;
		cmd_vel_msg.angular.z = angular_speed;
		cmd_vel_pub.publish(cmd_vel_msg);
	} else if (distance > 1) {
		// start a timer
		if (facing_obstacle) {
			++obstacle_timer;
			if (obstacle_timer > 20) { // distbug
				state = ROTATING;
				obstacle_timer = 0;
			}
		} else {
			obstacle_timer = 0;
		}
	} else {
		state = DETECTING;
		++waypoint_counter;
	}
}

/**
 * @param lidar_scan_msg ranges [m]: 0 -> 811, right -> left
 */
void lidar_callback(const sensor_msgs::LaserScan::ConstPtr& lidar_scan_msg) {
	if (manual) return;
	facing_obstacle = false;
	for (std::size_t i = LIDAR_FRONT_RIGHT; i <= LIDAR_FRONT_LEFT; ++i) {
		if (lidar_scan_msg->ranges[i] > 0.1 and lidar_scan_msg->ranges[i] < 1) {
			facing_obstacle = true;
		}
	}
	geometry_msgs::Twist cmd_vel_msg;
	std::size_t min_index;
	switch (state) {
	case DETECTING: {
		std::unordered_map<std::size_t, double> objects{};
		for (std::size_t i = 0; i < lidar_scan_msg->ranges.size(); ++i) {
			if (lidar_scan_msg->ranges[i] > 0.1) {
				double average = 0;
				std::size_t j = i;
				for (; j < lidar_scan_msg->ranges.size(); ++j) {
					average += lidar_scan_msg->ranges[j];
					if (lidar_scan_msg->ranges[j] - lidar_scan_msg->ranges[j + 1] > 0.5)
						break;
				}
				if (j - i >= 10) {
					objects[j > LIDAR_FRONT and i < LIDAR_FRONT ? LIDAR_FRONT
					                                            : (j + i) / 2] =
					   average / (j - i + 1);
				}
				i = j;
			}
		}
		for (auto&& [index, distance] : objects) {
			ROS_INFO("index=%ld, distance=%lf", index, distance);
		}
		auto const iter =
		   std::min_element(objects.cbegin(),
		                    objects.cend(),
		                    [](std::pair<std::size_t, double> const& lhs,
		                       std::pair<std::size_t, double> const& rhs) {
			                    return lhs.second < rhs.second;
		                    });
		min_index = std::distance(objects.cbegin(), iter);
		double distance =
		   std::sqrt(std::pow(objects[LIDAR_FRONT], 2) + std::pow(iter->second, 2)
		             - 2 * objects[LIDAR_FRONT] * iter->second
		                  * std::cos((LIDAR_FRONT - min_index) * lidar_scan_msg->angle_increment));
		ROS_INFO("index=%ld, object distance=%lf, cone distance=%lf, "
		         "distance=%lf",
		         min_index,
		         iter->second,
		         objects[LIDAR_FRONT],
		         distance);
	}
	case TURNING: {
		// turn to the way point
		if (min_index < LIDAR_FRONT) {
			ROS_INFO("Bucket is on the right of the cone");
			cmd_vel_msg.angular.z = -0.3;
		} else {
			ROS_INFO("Bucket is on the left of the cone");
			cmd_vel_msg.angular.z = 0.3;
		}
		cmd_vel_pub.publish(cmd_vel_msg);
		// finish with this waypoint
		if (facing_obstacle) {
			ROS_INFO("Turned to the bucket");
			std_msgs::Bool bucket;
			bucket.data = true;
			cv_pub.publish(bucket);
			state = DRIVING;
		}
	}
	case ROTATING: {
		cmd_vel_msg.angular.z = 0.3; // turn left
		cmd_vel_pub.publish(cmd_vel_msg);
		if (lidar_scan_msg->ranges[500] > 2) { // front right is open
			ROS_INFO("Start wall following\n");
			state = FOLLOWING;
		}
	}
	case FOLLOWING: {
		cmd_vel_msg.linear.x = 0.3; // drive straight
		cmd_vel_pub.publish(cmd_vel_msg);
		if (lidar_scan_msg->ranges[600] > 2) { // right is open
			ROS_INFO("Away from the obstacle\n");
			state = DRIVING;
		}
	}
	}
}

/**
 * @brief yaw: E: 0, N: +, S: -
 */
void imu_callback(const sensor_msgs::Imu::ConstPtr& imu_msg) {
	tf::Quaternion q(imu_msg->orientation.x,
	                 imu_msg->orientation.y,
	                 imu_msg->orientation.z,
	                 imu_msg->orientation.w);
	tf::Matrix3x3 m(q);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);
	current_heading = yaw;
}

int main(int argc, char** argv) {
	std::ifstream coordinate("../AUTO4508-Project/src/master_pkg/src/coordinate.csv");
	std::string line;
	if (not coordinate.is_open()) { return 1; }
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

	ros::init(argc, argv, "master");
	ros::NodeHandle n;

	ros::Subscriber joy_sub = n.subscribe("joy", 1000, joy_callback);
	ros::Subscriber gps_sub = n.subscribe("fix", 1000, gps_callback);
	ros::Subscriber imu_sub = n.subscribe("imu/data", 1000, imu_callback);
	ros::Subscriber lidar_sub =
	   n.subscribe("sick_tim_7xx/scan", 1000, lidar_callback);

	joy_pub = n.advertise<sensor_msgs::Joy>("master/joy", 1);
	cmd_vel_pub = n.advertise<geometry_msgs::Twist>("RosAria/cmd_vel", 1);
	cv_pub = n.advertise<std_msgs::Bool>("bucket", 1);

	ros::spin();
	return 0;
}
