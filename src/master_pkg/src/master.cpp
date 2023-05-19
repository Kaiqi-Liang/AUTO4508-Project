#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/Imu.h"
#include <tf/tf.h>
#include <vector>
#include "geometry_msgs/Twist.h"

struct Cartesian {
	double x;
	double y;
	double z;
};

bool manual = true;
bool facing_obstacle = false;
bool reached_waypoint = false;
std::size_t waypoint_counter = 0;
bool found_bucket = false;
ros::Publisher joy_pub;
ros::Publisher cmd_vel_pub;
std::vector<double> latitudes = {};
std::vector<double> longitudes = {};

// sensor_msgs::LaserScan::ConstPtr lidar_scan_msg;
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
	if (manual) return;
	double angular_speed = 0;
	Cartesian goal = ellip2cart(latitudes[waypoint_counter], longitudes[waypoint_counter]);
	Cartesian robot = ellip2cart(gps_fix_msg->latitude, gps_fix_msg->longitude);
	double distance = std::sqrt(std::pow(robot.x - goal.x, 2) + std::pow(robot.y - goal.y, 2) + std::pow(robot.z - goal.z, 2));
	double heading2goal = std::atan2(goal.y - robot.y, goal.x - robot.x);
	double angle = heading2goal - current_heading + M_PI / 2;
	if (std::abs(angle) > 0.3) {
		angular_speed = angle > 0 ? 0.3 : -0.3;
	}
	geometry_msgs::Twist cmd_vel_msg;
	ROS_INFO("distance = %lf angle = %lf facing_obstacle = %d", distance, angle, facing_obstacle);
	if (not facing_obstacle) {
		cmd_vel_msg.linear.x = 0.5;
		cmd_vel_msg.linear.y = 0;
		cmd_vel_msg.linear.z = 0;
		cmd_vel_msg.angular.x = 0;
		cmd_vel_msg.angular.y = 0;
		cmd_vel_msg.angular.z = angular_speed;
		cmd_vel_pub.publish(cmd_vel_msg);
	} else if (distance > 1) {
		// cmdvel_timeout = ros::Duration(5.0);
		if (facing_obstacle) {
			// distbug
		}
	}

	// if (distance < 1) {

	// 	double heading2bucket;
	// 	double heading2waypoint_cone;
	// 	double distance_waypoint;
	// 	double distance_bucket;

	// 	int bucketIdx_lidar = 10000;

	// 	// save original heading to waypoint cone
	// 	if (not reached_waypoint)
	// 	{ 
	// 		heading2waypoint_cone = current_heading;
	// 	 	reached_waypoint = true;		
	// 		waypoint_counter += 1;	// increase waypoint count
	// 	}
	// 	// call lidar values

	// 	while (bucketIdx_lidar == 10000)
	// 	{
	// 		//find lidar value smaller than 1.5 and not in the front -> bucket
	// 		bucketIdx_lidar = 500; // replace with find(lidar_scan_msg->ranges<1.5)  

	// 		// if there is a positive value (not in cone range) this is the bucket
	// 		if (bucketIdx_lidar != 10000 and std::abs(bucketIdx_lidar-405)>10) 
	// 		{
	// 			distance_bucket = lidar_scan_msg->ranges[bucketIdx_lidar]; // save distance to bucket
	// 			heading2bucket = current_heading;  // save heading to bucket
	// 		}
			
	// 		cmd_vel_msg.angular.z = 0.5;
	// 		cmd_vel_pub.publish(cmd_vel_msg);			
	// 	}

	// 	cmd_vel_msg.angular.z = 0;
	// 	cmd_vel_pub.publish(cmd_vel_msg);

	// 	// turn to bucket
	// 	if (std::abs(current_heading-heading2bucket) > 0.3) {
	// 		angular_speed = angle > 0 ? 0.3 : -0.3;
	// 		cmd_vel_msg.angular.z = angular_speed;
	// 		cmd_vel_pub.publish(cmd_vel_msg);	
	// 	}

	// 	// take a photo of bucket

	// 	// calculate distance between bucket and cone
	// 	double angle_coneBucket = std::abs(heading2bucket-heading2waypoint_cone);
	// 	double distance_coneBucket = std::sqrt(std::pow(distance_waypoint,2)+std::pow(distance_bucket,2)-2*distance_waypoint*distance_bucket*std::cos(angle_coneBucket));
	// 	std::cout<<"distance between waypoint "<<waypoint_counter<<" and bucket is "<<distance_coneBucket<<" m. /n";

	// 	// turn back to waypoint cone
	// 	if (std::abs(current_heading-heading2waypoint_cone) > 0.3) {
	// 		angular_speed = angle > 0 ? 0.3 : -0.3;
	// 		cmd_vel_msg.angular.z = angular_speed;
	// 		cmd_vel_pub.publish(cmd_vel_msg);	
	// 	}

	// 	if (heading2bucket-heading2waypoint_cone < 0 and distance_coneBucket < 1)
	// 	{
	// 		// obstacle = bucket
	// 	}
	// 	else
	// 	{
	// 		// obstacle = waypoint cone
	// 	}

	// 	// use distbug
	// }
}

void lidar_callback(const sensor_msgs::LaserScan::ConstPtr& lidar_scan_msg) {
	facing_obstacle = lidar_scan_msg->ranges[405] > 0;
}

// void cv_callback(auto const& cv_msg) {
// 	if (reached_waypoint) found_bucket = true;
// }

void imu_callback(const sensor_msgs::Imu::ConstPtr& imu_msg) {
	tf::Quaternion q(imu_msg->orientation.x, imu_msg->orientation.y, imu_msg->orientation.z, imu_msg->orientation.w);
	tf::Matrix3x3 m(q);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);
	current_heading = yaw;
	ROS_INFO("%lf %lf %lf", roll, pitch, yaw);
}

int main(int argc, char** argv) {
	std::ifstream waypoints("coordinate.csv");
	ros::init(argc, argv, "master");
	ros::NodeHandle n;
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
