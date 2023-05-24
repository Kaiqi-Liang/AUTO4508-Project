#include "geometry_msgs/Point.h"
#include "geometry_msgs/Twist.h"
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/Joy.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/NavSatFix.h"
#include "std_msgs/Float64.h"
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
	Coordinate() {}
	Coordinate(double latitude, double longitude)
	: latitude{latitude}
	, longitude{longitude} {}
};
Coordinate robot_gps;

enum State {
	DRIVING, // Driving towards the next waypoint (waypoint driving)
	ROTATING, // Rotating away for the obstacle after waiting for them to move (object avoidance)
	FOLLOWING, // Following the obstacle keeping it on the right (object avoidance)
	DETECTING, // Looking for the bucket after reaching a waypoint (bucket detection)
	TURNING, // Turning towards the bucket after detecting it (bucket detection)
	FINISHED, // Back to the first waypoint
};

constexpr std::size_t LIDAR_FRONT = 405;
constexpr std::size_t LIDAR_FRONT_RIGHT = 315;
constexpr std::size_t LIDAR_FRONT_LEFT = 495;

constexpr double SAFE_DISTANCE_WAYPOINT = 3;
constexpr double SAFE_DISTANCE_OBSTACLE = 2;
constexpr double ANGULAR_SPEED = 0.2;
constexpr double LINEAR_SPEED = 0.5;
constexpr double LIDAR_VALID_DISTANCE = 0.5;

bool deadman = true;
bool manual = true;
bool facing_obstacle = false;
bool found_bucket = false;
bool turn_right;

std::size_t waypoint_counter = 0;
std::size_t obstacle_timer = 0;

State state = DRIVING;
double heading = 0;
double bucket_cone_distance;
std::vector<Coordinate> coordinates;

ros::Publisher joy_pub;
ros::Publisher cmd_vel_pub;
ros::Publisher cv_pub;
ros::Publisher gps_pub;

void joy_callback(const sensor_msgs::Joy::ConstPtr& joy_msg) {
	if (joy_msg->buttons[1]) {
		manual = true;
	} else if (joy_msg->buttons[2]) { // automated mode
		manual = false;
	}
	deadman = not joy_msg->buttons[0];
	if (manual) joy_pub.publish(joy_msg);
}

Cartesian ellip2cart(double phi, double lambda) {
	phi *= M_PI / 180;
	lambda *= M_PI / 180;
	double axis = 6378137; // semi-major axis (WGS84) [m]
	double flattening = 1 / 298.257223563; // earth flattening (WGS84)
	double eccentricity =
	   std::sqrt((std::pow(axis, 2) - std::pow(axis * (1 - flattening), 2))
	             / std::pow(axis, 2));
	double height = 5;
	double radius_of_curvature =
	   axis
	   / std::sqrt(1 - std::pow(eccentricity, 2) * std::pow(std::sin(phi), 2));
	return {
	   (radius_of_curvature + height) * std::cos(phi) * std::cos(lambda),
	   (radius_of_curvature + height) * std::cos(phi) * std::sin(lambda),
	   (radius_of_curvature * (1 - std::pow(eccentricity, 2)) + height)
	      * std::sin(phi),
	};
}

void gps_callback(const sensor_msgs::NavSatFix::ConstPtr& gps_fix_msg) {
	// ROS_INFO("state=%d manual=%d", state, manual);
	if (deadman or manual or std::isnan(gps_fix_msg->latitude)
	    or std::isnan(gps_fix_msg->longitude) or state != DRIVING) {
		return;
	}

	robot_gps = {gps_fix_msg->latitude, gps_fix_msg->longitude};
	Cartesian goal = ellip2cart(coordinates[waypoint_counter].latitude,
	                            coordinates[waypoint_counter].longitude);
	Cartesian robot = ellip2cart(gps_fix_msg->latitude, gps_fix_msg->longitude);
	// ROS_INFO("x=%lf, y=%lf, z=%lf", goal.x, goal.y, goal.z);
	// ROS_INFO("x=%lf, y=%lf, z=%lf", robot.x, robot.y, robot.z);

	double distance =
	   std::sqrt(std::pow(robot.x - goal.x, 2) + std::pow(robot.y - goal.y, 2)
	             + std::pow(robot.z - goal.z, 2));
	double bearing = std::atan2(goal.x - robot.x, goal.y - robot.y);

	double turning_angle = bearing - heading + M_PI / 2;

	if (turning_angle > M_PI) {
		turning_angle -= 2 * M_PI;
	} else if (turning_angle < -M_PI) {
		turning_angle += 2 * M_PI;
	}

	double angular_speed = 0;
	if (std::abs(turning_angle) > 0.1) {
		if (std::abs(turning_angle) > 1) {
			angular_speed = turning_angle > 0 ? 0.8 : -0.8;
		} else if (std::abs(turning_angle) > 0.5) {
			angular_speed = turning_angle > 0 ? 0.5 : -0.5;
		} else if (std::abs(turning_angle) > 0.3) {
			angular_speed = turning_angle > 0 ? 0.3 : -0.3;
		} else {
			angular_speed = turning_angle > 0 ? 0.1 : -0.1;
		}
	}

	geometry_msgs::Twist cmd_vel_msg;
	if (distance > SAFE_DISTANCE_WAYPOINT and not facing_obstacle) {
		ROS_INFO("distance=%lf bearing=%lf turning_angle=%lf heading=%lf "
		         "facing_obstacle=%d",
		         distance,
		         bearing,
		         turning_angle,
		         heading,
		         facing_obstacle);
		cmd_vel_msg.linear.x = LINEAR_SPEED;
		cmd_vel_msg.angular.z = angular_speed;
		cmd_vel_pub.publish(cmd_vel_msg);
	} else if (distance > SAFE_DISTANCE_WAYPOINT) {
		if (facing_obstacle) {
			++obstacle_timer;
			if (obstacle_timer > 5) { // distbug
				geometry_msgs::Point obstacle_gps;
				double dist = SAFE_DISTANCE_OBSTACLE * 0.00001;
				obstacle_gps.x = gps_fix_msg->latitude + dist * std::sin(heading);
				obstacle_gps.y = gps_fix_msg->longitude + dist * std::cos(heading);
				obstacle_gps.z = 0;
				gps_pub.publish(obstacle_gps);
				state = ROTATING;
				obstacle_timer = 0;
			}
		} else {
			obstacle_timer = 0;
		}
	} else {
		state = DETECTING;
		++waypoint_counter;
		if (waypoint_counter == coordinates.size()) {
			ROS_INFO("Home sweet home\n");
			state = FINISHED;
		}
	}
}

/**
 * @param lidar_scan_msg ranges [m]: 0 -> 811, right -> left
 */
void lidar_callback(const sensor_msgs::LaserScan::ConstPtr& lidar_scan_msg) {
	if (deadman or manual) return;
	facing_obstacle = false;
	for (std::size_t i = LIDAR_FRONT_RIGHT; i <= LIDAR_FRONT_LEFT; ++i) {
		if (lidar_scan_msg->ranges[i] > 0.2
		    and lidar_scan_msg->ranges[i] < SAFE_DISTANCE_OBSTACLE) {
			facing_obstacle = true;
		}
	}
	geometry_msgs::Twist cmd_vel_msg;
	double bearing;
	switch (state) {
	case DETECTING: {
		std::unordered_map<std::size_t, double> objects{};
		for (std::size_t i = 0; i < lidar_scan_msg->ranges.size(); ++i) {
			if (lidar_scan_msg->ranges[i] > LIDAR_VALID_DISTANCE) {
				double average = 0;
				std::size_t j = i;
				for (; j < lidar_scan_msg->ranges.size(); ++j) {
					average += lidar_scan_msg->ranges[j];
					if (lidar_scan_msg->ranges[j] - lidar_scan_msg->ranges[j + 1] > 0.5)
						break;
				}
				if (j - i >= 10) {
					objects[(j > LIDAR_FRONT and i < LIDAR_FRONT) ? LIDAR_FRONT
					                                              : (j + i) / 2] =
					   average / (j - i + 1);
				}
				i = j;
			}
		}
		for (auto&& [index, distance] : objects) {
			ROS_INFO("index=%ld, distance=%lf", index, distance);
		}
		auto const cone_iter = std::min_element(
		   objects.cbegin(),
		   objects.cend(),
		   [](std::pair<std::size_t, double> const& lhs,
		      std::pair<std::size_t, double> const& rhs) {
			   return std::abs(int(LIDAR_FRONT) - int(lhs.first))
			          < std::abs(int(LIDAR_FRONT) - int(rhs.first));
		   });
		std::size_t cone_index = cone_iter->first;
		double cone_distance = cone_iter->second;
		objects.erase(cone_iter);
		auto const bucket_iter =
		   std::min_element(objects.cbegin(),
		                    objects.cend(),
		                    [](std::pair<std::size_t, double> const& lhs,
		                       std::pair<std::size_t, double> const& rhs) {
			                    return lhs.second < rhs.second;
		                    });
		std::size_t bucket_index = bucket_iter->first;
		double bucket_distance = bucket_iter->second;
		bucket_cone_distance =
		   std::sqrt(std::pow(cone_distance, 2) + std::pow(bucket_distance, 2)
		             - 2 * cone_distance * bucket_distance
		                  * std::cos(std::abs(int(cone_index) - int(bucket_index))
		                             * lidar_scan_msg->angle_increment));

		double turning_angle = std::abs(int(LIDAR_FRONT) - int(bucket_index))
		                       * lidar_scan_msg->angle_increment;
		turn_right = bucket_index < LIDAR_FRONT;
		if (turn_right) {
			ROS_INFO("Bucket is on the right of the cone");
			bearing = heading - turning_angle;
		} else {
			ROS_INFO("Bucket is on the left of the cone");
			bearing = heading + turning_angle;
		}
		bearing -= M_PI / 2;
		state = TURNING;
		geometry_msgs::Point bucket_gps;
		double dist = bucket_distance * 0.00001;
		bucket_gps.x = robot_gps.latitude + dist * std::sin(bearing);
		bucket_gps.y = robot_gps.longitude + dist * std::cos(bearing);
		bucket_gps.z = 1;
		gps_pub.publish(bucket_gps);
		ROS_INFO("turning_angle=%lf, heading=%lf, bearing=%lf, bucket_index=%ld, "
		         "cone_index=%ld, bucket_distance=%lf, cone_distance=%lf, "
		         "distance=%lf",
		         turning_angle,
		         heading,
		         bearing,
		         bucket_index,
		         cone_index,
		         bucket_distance,
		         cone_distance,
		         bucket_cone_distance);
		break;
	}
	case TURNING: {
		// turn to the way point
		if (std::abs(bearing - heading) < 0.3) {
			// finish with this waypoint
			ROS_INFO("Turned to the bucket");
			std_msgs::Float64 distance;
			distance.data = bucket_cone_distance;
			cv_pub.publish(distance);
			state = DRIVING;
			break;
		} else if (turn_right) {
			cmd_vel_msg.angular.z = -ANGULAR_SPEED;
		} else {
			cmd_vel_msg.angular.z = ANGULAR_SPEED;
		}
		cmd_vel_pub.publish(cmd_vel_msg);
		break;
	}
	case ROTATING: {
		cmd_vel_msg.angular.z = ANGULAR_SPEED; // turn left
		cmd_vel_pub.publish(cmd_vel_msg);
		bool front_open = true;
		for (std::size_t i = LIDAR_FRONT_RIGHT; i < LIDAR_FRONT; i += 5) {
			if (lidar_scan_msg->ranges[i] < 2 and lidar_scan_msg->ranges[i] > 0.2) {
				front_open = false;
			}
		}
		if (front_open) {
			ROS_INFO("Start wall following\n");
			state = FOLLOWING;
		}
		break;
	}
	case FOLLOWING: {
		if (facing_obstacle) state = DRIVING;
		cmd_vel_msg.linear.x = 1; // drive straight
		cmd_vel_pub.publish(cmd_vel_msg);
		bool right_open = true;
		for (std::size_t i = 0; i < 200; i += 5) {
			ROS_INFO("range[i]=%lf", lidar_scan_msg->ranges[i]);
			if (lidar_scan_msg->ranges[i] < 2 and lidar_scan_msg->ranges[i] > 0.2) {
				right_open = false;
			}
		}
		ROS_INFO("right_open=%d", right_open);
		if (right_open) {
			ROS_INFO("Away from the obstacle\n");
			state = DRIVING;
		}
		break;
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
	heading = yaw;
}

int main(int argc, char** argv) {
	std::ifstream coordinate("../AUTO4508-Project/src/master_pkg/src/coordinate.csv");
	std::string line;
	if (not coordinate.is_open()) return 1;
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
	coordinates.push_back(coordinates[0]);

	ros::init(argc, argv, "master");
	ros::NodeHandle n;

	ros::Subscriber joy_sub = n.subscribe("joy", 1000, joy_callback);
	ros::Subscriber gps_sub = n.subscribe("fix", 1000, gps_callback);
	ros::Subscriber imu_sub = n.subscribe("imu/data", 1000, imu_callback);
	ros::Subscriber lidar_sub =
	   n.subscribe("sick_tim_7xx/scan", 1000, lidar_callback);

	joy_pub = n.advertise<sensor_msgs::Joy>("master/joy", 1);
	cmd_vel_pub = n.advertise<geometry_msgs::Twist>("RosAria/cmd_vel", 1);
	cv_pub = n.advertise<std_msgs::Float64>("distance", 1);
	gps_pub = n.advertise<geometry_msgs::Point>("gps", 1);

	ros::spin();
	return 0;
}
