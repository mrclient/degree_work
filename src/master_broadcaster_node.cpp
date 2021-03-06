#include <ros/ros.h>
#include <vector>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

int main(int argc, char** argv){
	ros::init(argc, argv, "master_broadcaster_node");
	ros::NodeHandle nh;
	std::vector<double> master_pose;
	std::vector<std::string> list_of_agents;

	if(!nh.getParam	("/master_pose", master_pose)){
		ROS_ERROR_STREAM("File with master pose not found");
		ros::shutdown();
	}
	
	if(!nh.getParam("/list_of_agents", list_of_agents)){
		ROS_ERROR_STREAM("List of agents couldn't be read!");
		ros::shutdown();
	}
	
	ros::Duration(1.0).sleep();
	
	tf::TransformBroadcaster br;
	tf::Transform transform;	
	transform.setOrigin( tf::Vector3(master_pose[0], master_pose[1], 0.0) );	
	tf::Quaternion q;	
	q.setRPY(0.0, 0.0, master_pose[2]);
	transform.setRotation(q);

	ros::Rate loop_rate(5);
	while(ros::ok()){
		loop_rate.sleep();
		br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "NS_frame", list_of_agents[0]));	
	}
	return 0;
}
