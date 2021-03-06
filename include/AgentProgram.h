#ifndef AGENT_PROGRAM_H_
#define AGENT_PROGRAM_H_

#include <ros/ros.h>
#include <Robot.h>
#include <string>
#include <vector>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <cmath>

#define LOG_NAME "agent_node"

struct AgentState{
	tf::Vector3 coords;
};

class AgentProgram{

	ros::NodeHandle nh, private_nh;

	std::string agent_name;
	std::vector<std::string> list_of_agents;
	std::vector<float> aims;	
	double k_gain, distance_to_wall;
	bool use_northstar;
	bool obstacle_avoidance;

	Robot robot;
	unsigned int number_of_agents; 	
	std::vector<double> last_errors;

	std::vector<AgentState> agentsStates();
	tf::Vector3 calculateSpeed(std::vector<AgentState> agents_states, std::vector<tf::Vector3> signals_from_rangers);

public:
	AgentProgram();
	~AgentProgram();

	void run();
};

#endif /* AGENT_PROGRAM_H_ */
