#ifndef AGENT_PROGRAM_H_
#define AGENT_PROGRAM_H_

#include <ros/ros.h>
#include <string>
#include <vector>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/PointCloud.h>
#include <robotino_msgs/NorthStarReadings.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <cmath>
#include <fstream>
#include <iostream>

#define NUM_OF_RANGERS 9
#define ORDER_OF_FILTER 4


class AgentProgram{

	ros::NodeHandle nh, private_nh;

	ros::Publisher speeds_publisher;
	ros::Subscriber northstar_subscriber;
	ros::Subscriber dist_sens_subscriber;


	std::string agent_name, master_name;
	std::vector<float> aims;	
	double kr, kp, ki, kd, distance_to_wall;
	bool obstacle_avoidance;
	std::vector<tf::Vector3> signals_from_rangers;
	std::vector<float> master_pose;
	tf::Transform master_transform;
	std::ofstream fout, foutf;

	void northStarCallback(const robotino_msgs::NorthStarReadings &msg); 
	void distSensCallback(const sensor_msgs::PointCloud &msg);

	geometry_msgs::Twist calculateSpeed(double x, double y, double angle);
	void filter(const robotino_msgs::NorthStarReadings &msg, double &x, double &y, double &angle);

public:
	AgentProgram();
	~AgentProgram();

	void run();
};

#endif /* AGENT_PROGRAM_H_ */
