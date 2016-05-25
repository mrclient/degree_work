#ifndef SAFETY_NODE_H_
#define SAFETY_NODE_H_

#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_datatypes.h>
#include <string>
#include <vector>
#include <cmath>

#define NUM_OF_RANGERS 9
#define LOG_NAME "safety_node"

class SafetyNode{

	ros::NodeHandle nh, private_nh;
	ros::Publisher speed_publisher;
	ros::Subscriber cmd_subscriber;
	ros::Subscriber dist_sens_subscriber;
	
	double k_gain, distance_to_wall;
	std::vector<tf::Vector3> signals_from_rangers;

	void distSensCallback(const sensor_msgs::PointCloud &msg);
	void cmdCallback(const geometry_msgs::Twist &msg);
	
public:
	SafetyNode();
	~SafetyNode();

	void run();
};

#endif /* SAFETY_NODE_H_ */
