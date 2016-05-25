#include <ros/ros.h>
#include <Robot.h>
#include <string>

class BroadcastNSNode{
	ros::NodeHandle nh, private_nh;
	Robot robot;
	std::string robot_name;
public:
	BroadcastNSNode();
	~BroadcastNSNode();
	void run();
};


BroadcastNSNode::BroadcastNSNode() : private_nh("~"){
	if(!private_nh.getParam	("robot_name", robot_name)){
		ROS_ERROR_STREAM("Robot's name has not been specified!");
		ros::shutdown();
	}
	robot.getName(robot_name);
	robot.useNorthstar();
}


BroadcastNSNode::~BroadcastNSNode(){
}


void BroadcastNSNode::run(){
	ros::spin();
}


int main(int argc, char** argv){
	ros::init(argc, argv, "robot_program");
	BroadcastNSNode bn;
	ros::Duration(2.0).sleep();
	bn.run();
	return 0;
}
