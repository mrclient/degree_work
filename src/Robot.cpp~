#include <Robot.h>

Robot::Robot(){
	dist_sens_subscriber = nh.subscribe("distance_sensors", 1, &Robot::distSensCallback, this);
	speeds_publisher = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
	signals_from_rangers.resize(NUM_OF_RANGERS);
}



Robot::~Robot(){
	northstar_subscriber.shutdown();
	dist_sens_subscriber.shutdown();
	speeds_publisher.shutdown();
}



void Robot::useNorthstar(){
	northstar_subscriber = nh.subscribe("north_star", 1, &Robot::northStarCallback, this);
}



void Robot::getName(std::string nm){
	name = nm;
}



void Robot::distSensCallback(const sensor_msgs::PointCloud &msg){
	for(int i = 0; i < NUM_OF_RANGERS; i++){
		signals_from_rangers[i].setValue(msg.points[i].x, msg.points[i].y, 0.0);
	}
}


//write a transform between projector and "this agent"'s frame based on northstar's output
void Robot::northStarCallback(const robotino_msgs::NorthStarReadings &msg){
	static tf::TransformBroadcaster br;
	tf::Transform transform;
	transform.setOrigin( tf::Vector3(msg.pose.position.x, msg.pose.position.y, 0.0) );
	tf::Quaternion q;
	tf::quaternionMsgToTF(msg.pose.orientation, q);
	transform.setRotation(q);
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "NS_frame", name));
}



void Robot::move(tf::Vector3 speed){
	geometry_msgs::Twist msg;
	msg.linear.x = speed.x();
	msg.linear.y = speed.y();
	speeds_publisher.publish(msg);
}
