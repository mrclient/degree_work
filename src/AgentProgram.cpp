#include <AgentProgram.h>

AgentProgram::AgentProgram() : private_nh("~") {

	if(!private_nh.getParam	("agent_name", agent_name)){
		ROS_ERROR_STREAM("Agent name has not been specified!");
		ros::shutdown();
	}
	robot.getName(agent_name);

	if(!nh.getParam("/list_of_agents", list_of_agents)){
		ROS_ERROR_STREAM("List of agents couldn't be read!");
		ros::shutdown();
	}
	number_of_agents = list_of_agents.size();
	last_errors.resize(number_of_agents, 0.0);

	if(!private_nh.getParam	("aims", aims)){
		ROS_ERROR_STREAM("Needed robot's state has not been specified");
		ros::shutdown();
	}

	private_nh.param<double>("k_gain", k_gain, 2.0);
	private_nh.param<double>("distance_to_wall", distance_to_wall, 0.55);
	private_nh.param<bool>("obstacle_avoidance", obstacle_avoidance, false);
	private_nh.param<bool>("use_northstar", use_northstar, false);
	if(use_northstar) robot.useNorthstar();
	
}



AgentProgram::~AgentProgram(){
}


std::vector<AgentState> AgentProgram::agentsStates(){

	std::vector<AgentState> states(number_of_agents);	
	tf::TransformListener tf_listener;
	tf::StampedTransform transform;
	
	
	ROS_INFO_STREAM(nh.getNamespace() << " " << tf_listener.frameExists("projector"));
	//find robot's coordinates in "this agent" coordinate frame
	int j = 0;
	for(int i = 0; i < number_of_agents; i++){
		if(list_of_agents[i] != agent_name){		
			bool flag = true;
			while(ros::ok() && flag){
				try{
					flag = false;
					tf_listener.lookupTransform(agent_name, list_of_agents[i], ros::Time(0), transform);
				}
				catch (tf::TransformException ex){
					flag = true;
					ROS_ERROR_STREAM(nh.getNamespace() << " " << ex.what());
					ros::Duration(0.05).sleep();
				}
			}
			states[j++].coords = transform.getOrigin();
		}
	}
	ROS_INFO_STREAM("States was found");
	//State of this agent (useless information)
	states[j].coords = tf::Vector3(0.0, 0.0, 0.0);

	return states;
}



tf::Vector3 AgentProgram::calculateSpeed(std::vector<AgentState> agents_states, std::vector<tf::Vector3> signals_from_rangers){

	int i;
	static double last_time = 0.0;
	tf::Vector3 speed(0.0, 0.0, 0.0);
	double kp = 3.0, kd = 0.0;
	ros::Time curr_time;	
	double curr_error; 
	
	//formation maintaining control	
	curr_time = ros::Time::now();
	ROS_INFO_STREAM(curr_time.toSec());
	for(i = 0; i < (number_of_agents - 1); i++){
		curr_error = (agents_states[i].coords.length() - aims[i]);
		speed += (kp*curr_error+kd*(curr_error-last_errors[i])/(curr_time.toSec()-last_time))*agents_states[i].coords.normalized();		
		last_errors[i] = curr_error; 
	}
	last_time = curr_time.toSec();
	
	if(obstacle_avoidance){	
		//obstacles avoiding control
		if(speed.length() != 0.0){
			for(i = 0; i < signals_from_rangers.size(); i++){		
				double alpha;
				if(signals_from_rangers[i].length() < distance_to_wall){
					alpha = speed.angle(signals_from_rangers[i]);
					if(std::abs(alpha) < std::abs(M_PI / 2))
						speed -= signals_from_rangers[i].normalized() * speed.length() * std::cos(alpha);		
				}
			}
		}
	
		for(i = 0; i < signals_from_rangers.size(); i++){		
			if(signals_from_rangers[i].length() < distance_to_wall){
				speed -= signals_from_rangers[i].normalized() * k_gain * (distance_to_wall - signals_from_rangers[i].length());	
			}
		}
	}
	return speed;
}




void AgentProgram::run(){

	ros::Rate loop_rate(20);
	
	tf::TransformListener tf_listener;
	//Waiting for this agent's frame appearance
	while(ros::ok() && !tf_listener.frameExists(agent_name) && use_northstar){
		ros::spinOnce();
		loop_rate.sleep();
	}
	ros::Duration(4.0).sleep();

	//main program loop
	tf::Vector3 speed;
	while(ros::ok()){
 		speed = calculateSpeed(agentsStates(), robot.signals_from_rangers);		
		robot.move(speed);
		ros::spinOnce();
		loop_rate.sleep();
	}
}

