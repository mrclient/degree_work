#include <AgentProgram.h>
#include <fstream>
#include <iostream>
#include <iomanip>

std::ofstream fout;

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

	if(!private_nh.getParam	("aims", aims)){
		ROS_ERROR_STREAM("Needed robot's state has not been specified");
		ros::shutdown();
	}

	private_nh.param<double>("k_gain", k_gain, 2.0);
	private_nh.param<double>("distance_to_wall", distance_to_wall, 0.55);
	private_nh.param<bool>("obstacle_avoidance", obstacle_avoidance, false);
	private_nh.param<bool>("use_northstar", use_northstar, false);
	if(use_northstar) robot.useNorthstar();
	
	ROS_INFO_STREAM(agent_name);
	if(agent_name == "robot_1/base_link")	
		fout.open("/home/evgeniy/rotat1.txt");
	else
		fout.open("/home/evgeniy/rotat2.txt");
	//fout.open("/home/evgeniy/sim_line.txt");
	fout << std::setiosflags(std::ios::scientific);
}



AgentProgram::~AgentProgram(){
}


std::vector<AgentState> AgentProgram::agentsStates(){

	std::vector<AgentState> states(number_of_agents);	
	tf::TransformListener tf_listener;
	tf::StampedTransform transform;
	
	
	bool flag = true;
	while(ros::ok() && flag){
		try{
			flag = false;
			tf_listener.lookupTransform(list_of_agents[0], agent_name, ros::Time(0), transform);
		}
		catch (tf::TransformException ex){
			flag = true;
			//ROS_ERROR_STREAM(nh.getNamespace() << " " << ex.what());
			ros::Duration(0.05).sleep();
		}
	}
	states[0].coords = transform.getOrigin();

	return states;
}


#define IMAX 1.0
#define IMIN (-IMAX)

tf::Vector3 AgentProgram::calculateSpeed(std::vector<AgentState> agents_states, std::vector<tf::Vector3> signals_from_rangers){

	int i;
	double kp = 3.0, kd = 0.5, ki = 3.0; //kp = 2.0, kd = 0.05, ki = 0.5;//
	static double ux = 0.0, uy = 0.0, x[3] = {0.0, 0.0, 0.0}, y[3] = {0.0, 0.0, 0.0};
	static double ix = 0.0, iy = 0.0;
	double Ix, Iy;	
	//const double T = 0.1;	
	ros::Time curr_time;
	static double last_time = ros::Time::now().toSec();

	geometry_msgs::Vector3Stamped global_speed, local_speed;
	tf::TransformListener tf_listener;

	curr_time = ros::Time::now();
	//formation maintaining control	
	x[2] = x[1];
	x[1] = x[0];	
	x[0] = aims[0] - agents_states[0].coords.x();
	
	if(ix + ki*0.105*x[0] > IMAX)
		Ix = IMAX;
	else if(ix + ki*0.105*x[0] < IMIN)
		Ix = IMIN;
	else
		Ix = ix + ki*0.105*x[0];
//	ROS_INFO_STREAM(curr_time.toSec() - last_time);
	ROS_INFO_STREAM(Ix);
	global_speed.vector.x = ux + (kp + kd) * x[0] - (kp + 2 * kd) * x[1] + kd * x[2] + Ix - ix;
	ix = Ix;	

	y[2] = y[1];
	y[1] = y[0];
	y[0] = aims[1] - agents_states[0].coords.y();
	if(iy + ki*0.105*y[0] > IMAX)
		Iy = IMAX;
	else if(iy + ki*0.105*y[0] < IMIN)
		Iy = IMIN;
	else
		Iy = iy + ki*0.105*y[0];
	global_speed.vector.y = uy + (kp + + kd) * y[0] - (kp + 2 * kd) * y[1] + kd * y[2] + Iy - iy;		
	iy = Iy;	
	last_time = curr_time.toSec();

	ux = global_speed.vector.x;
	uy = global_speed.vector.y;

	global_speed.header.frame_id = list_of_agents[0];
	bool flag = true;
	while(ros::ok() && flag){
		try{
			flag = false;
			tf_listener.transformVector(agent_name, global_speed, local_speed);
		}
		catch (tf::TransformException ex){
			flag = true;
			//ROS_ERROR_STREAM(nh.getNamespace() << " " << ex.what());
			ros::Duration(0.05).sleep();
		}
	}
	
	static double time = 0.0;
	fout << x[0] << " " << y[0] << " " << time << std::endl;
	time += 0.105; 

	tf::Vector3 speed;
	speed = tf::Vector3(local_speed.vector.x, local_speed.vector.y, 0.0);

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

	ros::Rate loop_rate(100);
	
	tf::TransformListener tf_listener;
	//Waiting for "this agent's" frame's appearance
	while(ros::ok() && !tf_listener.frameExists(agent_name)){
		ros::spinOnce();
		loop_rate.sleep();
	}
	//ros::Duration(2.0).sleep();

	//main program loop
	tf::Vector3 speed;
	while(ros::ok()){
 		speed = calculateSpeed(agentsStates(), robot.signals_from_rangers);		
		robot.move(speed);
		ros::spinOnce();
		loop_rate.sleep();
	}
	fout.close();
}

