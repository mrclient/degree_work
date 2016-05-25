#include <SafetyNode.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "safety_node");
	SafetyNode sn;
	sn.run();
	return 0;
}
