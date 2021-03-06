#include <IdentificationNode.h>

#define	QUATER_SIZE 25

IdentificationProgram::IdentificationProgram() : private_nh("~"), angle0(0.0) {

	if(!nh.getParam("/list_of_agents", list_of_agents)){
		ROS_ERROR_STREAM("List of agents couldn't be read!");
		ros::shutdown();
	}

	if(!private_nh.getParam	("agent_name", agent_name)){
		ROS_ERROR_STREAM("Agent name has not been specified!");
		ros::shutdown();
	}
	
	strobe = false;
	speeds_publisher = nh.advertise<geometry_msgs::Twist>(list_of_agents[0] + "/cmd_vel", 1);
	northstar_subscriber = nh.subscribe(agent_name + "/north_star", 1, &IdentificationProgram::northStarCallback, this);
	
	x.resize(4*QUATER_SIZE);
	y.resize(4*QUATER_SIZE);
	angle.resize(4*QUATER_SIZE);
}



IdentificationProgram::~IdentificationProgram(){
	northstar_subscriber.shutdown();
	speeds_publisher.shutdown();
}



void IdentificationProgram::northStarCallback(const robotino_msgs::NorthStarReadings &msg){
	static int counter = 0;
	static bool flag = true;
	static double cur_angle, last_angle, msg_angle;	
	static int index = 0;	

	ROS_INFO_STREAM("In callback");
	if(flag){
		cur_angle = tf::getYaw(msg.pose.orientation);
		last_angle = cur_angle;
		flag = false;	
	}
	else{
		msg_angle = tf::getYaw(msg.pose.orientation);
		if( ((msg_angle - last_angle) < M_PI) && ((msg_angle - last_angle) > -M_PI)){
			cur_angle += msg_angle - last_angle;
		}
		else if((msg_angle - last_angle) > M_PI){
			cur_angle += (msg_angle - last_angle) - 2 * M_PI;
		}
		else{
			cur_angle += (msg_angle - last_angle) + 2 * M_PI;		
		}	
		last_angle = msg_angle;		

		if(strobe){
			x[index % (4*QUATER_SIZE)] = msg.pose.position.x;
			y[index % (4*QUATER_SIZE)] = msg.pose.position.y;
			angle[index % (4*QUATER_SIZE)] = cur_angle - angle0;
			fout << msg.pose.position.x << " "  << msg.pose.position.y << " "  << cur_angle - angle0 <<"\n";
			index++;			
			counter++;	
			if(counter == number_of_values){
				counter = 0;
				strobe = false;			
			}
		}
	}
}



void IdentificationProgram::makeMove(geometry_msgs::Twist msg){
	strobe = true;	
	ros::Rate loop_rate(10);
	while( ros::ok() && strobe){
		speeds_publisher.publish(msg);					
		ros::spinOnce();
		loop_rate.sleep();
	}
}



void IdentificationProgram::stopRobot(){
	geometry_msgs::Twist speed;
	ros::Rate loop_rate(10);
	//this magic value (5) is used only in this function
	for(int i = 0; i < 5; i++){
		speeds_publisher.publish(speed);
		ros::spinOnce();
		loop_rate.sleep();
	}
}

void IdentificationProgram::run(){

	int i;
	geometry_msgs::Twist speed;

	ros::Duration(6.0).sleep();
	
	ROS_INFO_STREAM("Program has started");
	number_of_values = 4*QUATER_SIZE;	
	makeMove(speed);

	fout.open("/home/evgeniy/for_ros/my_ws/src/degree_work/config/IP_rotat_data.txt");
	double x0 = 0, y0 = 0;
	for(i = 0; i < 4*QUATER_SIZE; i++){
		x0 += x[i];
		y0 += y[i];
		angle0 += angle[i];
	}
	x0 /= 4*QUATER_SIZE;
	y0 /= 4*QUATER_SIZE;
	angle0 /= 4*QUATER_SIZE;	
	fout << x0 << " "  << y0 << " "  << 0.0 <<"\n";

	ROS_INFO_STREAM("Starting rotations");
	//clockwise rotation
	number_of_values = QUATER_SIZE;	
	speed.angular.z = -0.5;
	makeMove(speed);
	stopRobot();
	
	//counterclockwise rotation
	number_of_values = 2*QUATER_SIZE;	
	speed.angular.z = 0.5;
	makeMove(speed);
	stopRobot();

	//clockwise rotation
	number_of_values = QUATER_SIZE;	
	speed.angular.z = -0.5;
	makeMove(speed);
	stopRobot();
	
	fout.close();
	speed.angular.z = 0.0;
	std::system("scilab-cli -f /home/evgeniy/for_ros/my_ws/src/degree_work/scilab_scripts/find_place.sce");	

	number_of_values = 4*QUATER_SIZE;	
	makeMove(speed);

	fout.open("/home/evgeniy/for_ros/my_ws/src/degree_work/config/IP_lin_data.txt");
	x0 = 0; 
	y0 = 0;
	angle0 = 0;
	for(i = 0; i < 4*QUATER_SIZE; i++){
		x0 += x[i];
		y0 += y[i];
	}
	x0 /= 4*QUATER_SIZE;
	y0 /= 4*QUATER_SIZE;
	fout << x0 << " "  << y0 << " "  << 0.0 <<"\n";

	//forward move
	number_of_values = QUATER_SIZE;	
	speed.linear.x = 0.1;
	makeMove(speed);
	stopRobot();
	
	//back move
	number_of_values = 2*QUATER_SIZE;	
	speed.linear.x = -0.1;
	makeMove(speed);
	stopRobot();

	//forward move
	number_of_values = QUATER_SIZE;	
	speed.linear.x = 0.1;
	makeMove(speed);
	stopRobot();

	fout.close();
	speed.linear.x = 0.0;
	std::system("scilab-cli -f /home/evgeniy/for_ros/my_ws/src/degree_work/scilab_scripts/find_orient.sce");

	ros::shutdown();
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "identification_program");
	IdentificationProgram ip;
	ip.run();
	return 0;
}
