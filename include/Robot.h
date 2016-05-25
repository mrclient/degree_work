#ifndef ROBOT_CLASS_H_
#define ROBOT_CLASS_H_

#include <ros/ros.h>
#include <robotino_msgs/NorthStarReadings.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/PointCloud.h>
#include <string>
#include <vector>
#include <cmath>

#define NUM_OF_RANGERS 9

class Robot{

	ros::NodeHandle nh;
	ros::Publisher speeds_publisher;
	ros::Subscriber northstar_subscriber;
	ros::Subscriber dist_sens_subscriber;
	
	std::string name;	

	void northStarCallback(const robotino_msgs::NorthStarReadings &msg); 
	void distSensCallback(const sensor_msgs::PointCloud &msg);

public:
	Robot();
	~Robot();

	std::vector<tf::Vector3> signals_from_rangers;

	void move(tf::Vector3 speed);
	void getName(std::string nm);
	void useNorthstar();
};

#endif /* ROBOT_CLASS_H_ */
