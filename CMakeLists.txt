cmake_minimum_required(VERSION 2.8.3)
project(slam_main)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS 
	thread 
	system
)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS} ${CMAKE_CXX_FLAGS_RELEASE}")

catkin_package(
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_executable(slam_main_node
	src/helper.cpp
	src/landmark.cpp
	src/keypoint_filter.cpp
	src/nn_finder.cpp
	src/feature_associator.cpp
	src/feature_associator_nn.cpp
	src/factor_graph.cpp 
	src/slam_main_node.cpp)

target_link_libraries(slam_main_node
	${catkin_LIBRARIES} 
	${OpenCV_LIBRARIES}
	${Boost_LIBRARIES}
	tbb
	tbbmalloc
	gtsam
)

# add_dependencies(slam_main_node slam_main_generate_messages_cpp)
