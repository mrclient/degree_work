#ifndef IDENTIFICATION_PROGRAM_H_
#define IDENTIFICATION_PROGRAM_H_

#include <ros/ros.h>
#include <Robot.h>
#include <string>
#include <vector>
#include <cmath>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Twist.h>
#include <cstdlib>
#include <fstream>
#include <iostream>

#define LOG_NAME "agent_node"

class IdentificationProgram{

	ros::NodeHandle nh, private_nh;
	std::vector<std::string> list_of_agents;
	std::string agent_name;	
	bool strobe;
	int number_of_values;
	std::vector<double> x, y, angle;
	ros::Publisher speeds_publisher;	
	ros::Subscriber northstar_subscriber;
	double angle0;
	std::ofstream fout;
	
	void northStarCallback(const robotino_msgs::NorthStarReadings &msg);	
	void makeMove(geometry_msgs::Twist msg);
	void stopRobot();
	

public:
	IdentificationProgram();
	~IdentificationProgram();

	void run();
};

#endif /* IDENTIFICATION_PROGRAM_H_ */
