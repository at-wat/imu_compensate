cmake_minimum_required(VERSION 2.8.3)
project(imu_compensate)

find_package(catkin REQUIRED COMPONENTS
	roscpp tf sensor_msgs std_msgs nav_msgs trajectory_msgs geometry_msgs
)

catkin_package(
	CATKIN_DEPENDS roscpp tf sensor_msgs std_msgs nav_msgs trajectory_msgs geometry_msgs
	DEPENDS system_lib
)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
	message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()


include_directories(${catkin_INCLUDE_DIRS} src)

add_executable(imu_compensate src/imu_compensate.cpp)
target_link_libraries(imu_compensate ${catkin_LIBRARIES})

add_executable(dummy_imu src/dummy_imu.cpp)
target_link_libraries(dummy_imu ${catkin_LIBRARIES})
