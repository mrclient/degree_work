#include <AgentProgram.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "agent_program");
	AgentProgram ap;
	ap.run();
	return 0;
}
