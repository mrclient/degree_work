cmake_minimum_required(VERSION 2.8.3)
project(degree_work)

find_package(catkin REQUIRED COMPONENTS geometry_msgs sensor_msgs robotino_msgs roscpp tf)

catkin_package()

include_directories(
	include
	${catkin_INCLUDE_DIRS}	
)

add_executable(
	agent_program 
	src/AgentProgram_P_regul.cpp
	src/agent_program.cpp
	src/Robot.cpp)

target_link_libraries(agent_program ${catkin_LIBRARIES})

add_executable(
	safety_node 
	src/safety_node.cpp
	src/SafetyNode.cpp)

target_link_libraries(safety_node ${catkin_LIBRARIES})

add_executable(
	broadcastNS_node 
	src/broadcast_northstar_node.cpp
	src/Robot.cpp)

target_link_libraries(broadcastNS_node ${catkin_LIBRARIES})

add_executable(
	identification_node 
	src/IdentificationNode.cpp)

target_link_libraries(identification_node ${catkin_LIBRARIES})

add_executable(
	master_broadcaster_node 
	src/master_broadcaster_node.cpp)


target_link_libraries(master_broadcaster_node ${catkin_LIBRARIES})

add_executable(
	simple_node 
	src/AgentProgram_simple.cpp)

target_link_libraries(simple_node ${catkin_LIBRARIES})
