#include <AgentProgram_simple.h>

void shift_right(double *array, int n){
	for(int i = 1; i < n; i++){
		array[n-i] = array[n-i-1];
	}
}

AgentProgram::AgentProgram() : private_nh("~") {

	if(!private_nh.getParam	("agent_name", agent_name)){
		ROS_ERROR_STREAM("Agent name has not been specified!");
		ros::shutdown();
	}

	if(!private_nh.getParam	("master_name", master_name)){
		ROS_ERROR_STREAM("Agent name has not been specified!");
		ros::shutdown();
	}


	if(!private_nh.getParam	("aims", aims)){
		ROS_ERROR_STREAM("Needed robot's state has not been specified");
		ros::shutdown();
	}

	if(!nh.getParam	("/master_pose", master_pose)){
		ROS_ERROR_STREAM("Needed master's pose has not been specified");
		ros::shutdown();
	}

	master_transform.setOrigin(tf::Vector3(master_pose[0], master_pose[1], 0.0));
	tf::Quaternion q;
	q.setRPY(0.0, 0.0, master_pose[2]);
	master_transform.setRotation(q);

	private_nh.param<double>("KR", kr, 1.0);
	private_nh.param<double>("KP", kp, 1.0);
	private_nh.param<double>("KI", ki, 1.0);
	private_nh.param<double>("KD", kd, 1.0);
	private_nh.param<double>("distance_to_wall", distance_to_wall, 0.485);
	private_nh.param<bool>("obstacle_avoidance", obstacle_avoidance, false);

	dist_sens_subscriber = nh.subscribe("distance_sensors", 1, &AgentProgram::distSensCallback, this);
	speeds_publisher = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
	signals_from_rangers.resize(NUM_OF_RANGERS);
	northstar_subscriber = nh.subscribe("north_star", 1, &AgentProgram::northStarCallback, this);	

	std::string s;
	s = "/home/evgeniy/robotino_exps/err"+agent_name+".txt";
	fout.open(s.c_str());
//	std::string s;
	//s = "/home/evgeniy/for_ros/my_ws/src/degree_work/config/filt"+agent_name+".txt";
	//foutf.open(s.c_str());
		
}

AgentProgram::~AgentProgram(){
	fout.close();
	//foutf.close();
	dist_sens_subscriber.shutdown();
	speeds_publisher.shutdown();
	northstar_subscriber.shutdown();
}


void AgentProgram::distSensCallback(const sensor_msgs::PointCloud &msg){
	for(int i = 0; i < NUM_OF_RANGERS; i++){
		signals_from_rangers[i].setValue(msg.points[i].x, msg.points[i].y, 0.0);
	}
}


void AgentProgram::northStarCallback(const robotino_msgs::NorthStarReadings &msg){
	
	static int counter = 0;
	static double last_time = 0.0; 
	double x, y, angle;
	filter(msg, x, y, angle);

	if(counter > 20){
			
		tf::Quaternion q;
		q.setRPY(0.0, 0.0, angle);
	
		tf::Transform in_master_frame;

		in_master_frame = master_transform.inverse() * tf::Transform(q, tf::Vector3(x, y, 0.0));

		geometry_msgs::Twist speed;
		speed=calculateSpeed(in_master_frame.getOrigin().x(),in_master_frame.getOrigin().y(),tf::getYaw(in_master_frame.getRotation()));	

		speeds_publisher.publish(speed);
	}
	else
		counter++;
}



void AgentProgram::filter(const robotino_msgs::NorthStarReadings &msg, double &x, double &y, double &angle){
	static double x_out[ORDER_OF_FILTER+1] = {0.0, 0.0, 0.0, 0.0, 0.0};
	static double x_in[ORDER_OF_FILTER+1] = {0.0, 0.0, 0.0, 0.0, 0.0};
	static double y_out[ORDER_OF_FILTER+1] = {0.0, 0.0, 0.0, 0.0, 0.0};
	static double y_in[ORDER_OF_FILTER+1] = {0.0, 0.0, 0.0, 0.0, 0.0};
	static double angle_out[ORDER_OF_FILTER+1] = {0.0, 0.0, 0.0, 0.0, 0.0};
	static double angle_in[ORDER_OF_FILTER+1] = {0.0, 0.0, 0.0, 0.0, 0.0};
	static bool flag = true;
	static double cur_angle, last_msg_angle, msg_angle;	
	
	static double b[ORDER_OF_FILTER+1] = {0.02, 0.082, 0.12, 0.082, 0.020};
	static double a[ORDER_OF_FILTER+1] = {0.0, -1.5, 1.2, -0.45, 0.07};

	shift_right(x_in, ORDER_OF_FILTER+1);
	shift_right(y_in, ORDER_OF_FILTER+1);
	shift_right(angle_in, ORDER_OF_FILTER+1);
	shift_right(x_out, ORDER_OF_FILTER+1);
	shift_right(y_out, ORDER_OF_FILTER+1);
	shift_right(angle_out, ORDER_OF_FILTER+1);

	x_in[0] = msg.pose.position.x;
	y_in[0] = msg.pose.position.y;
	
	if(flag){
		cur_angle =  tf::getYaw(msg.pose.orientation);
		last_msg_angle = cur_angle; 		
		flag = false;	
	}
	else{
		msg_angle = tf::getYaw(msg.pose.orientation);
		if( ((msg_angle - last_msg_angle) < M_PI) && ((msg_angle - last_msg_angle) > -M_PI)){
			cur_angle += msg_angle - last_msg_angle;
		}
		else if((msg_angle - last_msg_angle) > M_PI){
			cur_angle += (msg_angle - last_msg_angle) - 2 * M_PI;
		}
		else{
			cur_angle += (msg_angle - last_msg_angle) + 2 * M_PI;		
		}	
		last_msg_angle = msg_angle;
	}
	angle_in[0] = cur_angle;

	x_out[0] = 0.0;
	y_out[0] = 0.0;
	angle_out[0] = 0.0;

	x = 0;
	y = 0;
	angle = 0;
	for(int i = 0; i <= ORDER_OF_FILTER; i++){
		x += b[i] * x_in[i] - a[i]*x_out[i];
		y += b[i] * y_in[i] - a[i]*y_out[i];
		angle += b[i] * angle_in[i] - a[i]*angle_out[i];	
	}
	x_out[0] = x;
	y_out[0] = y;
	angle_out[0] = angle;
	foutf << x_in[0] << " " << y_in[0] << " " << x << " " << y << " " << angle_in[0] << " " << angle << " " << std::endl;
}





#define Imax 1.0
#define Imin (-Imax)

geometry_msgs::Twist AgentProgram::calculateSpeed(double x, double y, double angle){

	int i;
	static double ex[3] = {0.0, 0.0, 0.0}, ey[3] = {0.0, 0.0, 0.0};
	static double last_Ix = 0.0, last_Iy = 0.0, last_Dx = 0.0, last_Dy = 0.0, last_Px = 0.0, last_Py = 0.0; 
	double Ix, Iy, Dx, Dy, Px, Py;
	const double T = 0.14;	
	ros::Time curr_time;
	static double last_time = ros::Time::now().toSec();
	tf::Vector3 global_speed, local_speed;;


	curr_time = ros::Time::now();

	shift_right(ex, 3);
	ex[0] = aims[0] - x;
	fout << ex[0] << " ";
	if((last_Ix + ki * T * ex[0]) > Imax)
		Ix = Imax;
	else if ((last_Ix + ki * T * ex[0]) < Imin)
		Ix = Imin;
	else
		Ix = last_Ix + ki * T * ex[0];
	last_Ix = Ix;	

	Px = last_Px + kp * (ex[0] - ex[1]);
	last_Px = Px;
	Dx = last_Dx + kd/T * (ex[0] - 2*ex[1] + ex[2]);
	last_Dx = Dx;
	global_speed.setX(Px + Dx + Ix);

	
	shift_right(ey, 3);
	ey[0] = aims[1] - y;
	fout << ey[0] << std::endl;
	if((last_Iy + ki * T * ey[0]) > Imax)
		Iy = Imax;
	else if ((last_Iy + ki * T * ey[0]) < Imin)
		Iy = Imin;
	else
		Iy = last_Iy + ki * T * ey[0];
	last_Iy = Iy;	

	Py = last_Py + kp * (ey[0] - ey[1]);
	last_Py = Py;
	Dy = last_Dy + kd/T * (ey[0] - 2*ey[1] + ey[2]);
	last_Dy = Dy;
	global_speed.setY(Py + Dy + Iy);	
	
	
//	ROS_INFO_STREAM(curr_time.toSec() - last_time);
	last_time = curr_time.toSec();

	
	local_speed.setX(global_speed.x()*std::cos(angle) + global_speed.y()*std::sin(angle));
	local_speed.setY(-global_speed.x()*std::sin(angle) + global_speed.y()*std::cos(angle));

	tf::Vector3 speed;
	speed = local_speed;

	if(obstacle_avoidance){	
		//obstacles avoiding control
		/*if(speed.length() != 0.0){
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
				speed -= signals_from_rangers[i].normalized() * kr * (distance_to_wall - signals_from_rangers[i].length());	
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
	}
	geometry_msgs::Twist msg;
	msg.linear.x = speed.x();
	msg.linear.y = speed.y();
	return msg;
}




void AgentProgram::run(){
	ros::spin();
}


int main(int argc, char** argv){
	ros::init(argc, argv, "agent_program");
	AgentProgram ap;
	ap.run();
	return 0;
}

