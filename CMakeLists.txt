cmake_minimum_required(VERSION 2.8.3)
project(gologpp_agent)


add_definitions(-DSOURCE_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}\")
find_package(PkgConfig)
## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

pkg_check_modules(readylogpp REQUIRED readylog++)
pkg_check_modules(parsegologpp REQUIRED parsegolog++)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  genmsg
  message_generation
  actionlib_msgs
  actionlib
  naoqi_wrapper_msgs
  naoqi_bridge_msgs
  darknet_action_msgs
)

add_definitions(-DLIBQI_VERSION=${naoqi_libqi_VERSION_MAJOR}${naoqi_libqi_VERSION_MINOR})
if(CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated-declarations -Wno-placement-new")
endif()


include_directories(
  ${catkin_INCLUDE_DIRS}
  ${readylogpp_INCLUDE_DIRS}
  ${parsegologpp_INCLUDE_DIRS}
)
catkin_package(
  #INCLUDE_DIRS include
  #LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS actionlib_msgs actionlib naoqi_wrapper_msgs
  #CATKIN_DEPENDS actionlib actionlib_msgs message_generation roscpp rospy std_msgs
  #DEPENDS system_lib
)
set(BUILD_TESTS OFF CACHE BOOL "Enable gologpp tests" FORCE)

link_directories(${readylogpp_LIBRARY_DIRS} ${parsegologpp_LIBRARY_DIRS})

add_executable(gologpp_agent
    src/agent_node.cpp
    src/ros_backend.cpp
    src/pepper_actions.cpp
    src/action_manager.cpp
)
set_property(TARGET gologpp_agent PROPERTY CXX_STANDARD 14)

#add_dependencies(naoqi_say_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


target_link_libraries(
    gologpp_agent
    ${catkin_LIBRARIES}
    ${readylogpp_LIBRARIES}
    ${parsegologpp_LIBRARIES}
)

