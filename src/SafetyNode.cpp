#include <SafetyNode.h>

SafetyNode::SafetyNode() : private_nh("~"){
	if(!private_nh.getParam("k_gain", k_gain)){
		ROS_ERROR("%s: Gain of regulator has not been specified", LOG_NAME);
		ros::shutdown();
	}

	if(!private_nh.getParam("distance_to_wall", distance_to_wall)){
		ROS_ERROR("%s: Needed distance to wall has not been specified!", LOG_NAME);
		ros::shutdown();
	}

	cmd_subscriber = nh.subscribe("safe_cmd_vel", 1, &SafetyNode::cmdCallback, this);
	dist_sens_subscriber = nh.subscribe("distance_sensors", 1, &SafetyNode::distSensCallback, this);
	speed_publisher = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
	
	signals_from_rangers.resize(NUM_OF_RANGERS);

	ROS_INFO("%s: %s", LOG_NAME, "Node has started successfully.");
}



SafetyNode::~SafetyNode(){
	cmd_subscriber.shutdown();
	dist_sens_subscriber.shutdown();
	speed_publisher.shutdown();
}




void SafetyNode::distSensCallback(const sensor_msgs::PointCloud &msg){
	for(int i = 0; i < NUM_OF_RANGERS; i++)
		signals_from_rangers[i].setValue(msg.points[i].x, msg.points[i].y, 0.0);
}




void SafetyNode::cmdCallback(const geometry_msgs::Twist &msg){

	tf::Vector3 speed;
	speed.setValue(msg.linear.x, msg.linear.y, 0.0);
	
	int i;
	
	/*if(speed.length() != 0.0){
		for(i = 0; i < signals_from_rangers.size(); i++){		
			double alpha;
			if(signals_from_rangers[i].length() < distance_to_wall){
				alpha = speed.angle(signals_from_rangers[i]);
				if(std::abs(alpha) < std::abs(M_PI / 2))
					speed -= signals_from_rangers[i].normalized() * ( k_gain * (distance_to_wall - 
						 signals_from_rangers[i].length()) + speed.length() * std::cos(alpha) );	
				else
					speed -= signals_from_rangers[i].normalized() * k_gain * (distance_to_wall - 
						 signals_from_rangers[i].length());	
			}
		}
	}
	else{
		for(i = 0; i < signals_from_rangers.size(); i++){		
			if(signals_from_rangers[i].length() < distance_to_wall){
				speed -= signals_from_rangers[i].normalized() * k_gain * (distance_to_wall - signals_from_rangers[i].length());	
			}
		}
	}*/

	if(speed.length() != 0.0){
		for(i = 0; i < signals_from_rangers.size(); i++){		
			double alpha;
			if(signals_from_rangers[i].length() < distance_to_wall){
				alpha = speed.angle(signals_from_rangers[i]);
				if(std::abs(alpha) < std::abs(M_PI / 2))
					speed -= signals_from_rangers[i].normalized() * speed.length() * std::cos(alpha) * 
							(distance_to_wall - signals_from_rangers[i].length()) / distance_to_wall;		
			}
		}
	}
	
	for(i = 0; i < signals_from_rangers.size(); i++){		
		if(signals_from_rangers[i].length() < distance_to_wall){
			speed += signals_from_rangers[i].normalized() * (1.0/distance_to_wall) * 
									(signals_from_rangers[i].length() - distance_to_wall);	
		}
	}

	geometry_msgs::Twist safe_msg;
	safe_msg.linear.x = speed.x();
	safe_msg.linear.y = speed.y();
	safe_msg.angular = msg.angular;
	speed_publisher.publish(safe_msg);
}



void SafetyNode::run(){
	ros::spin();
}

