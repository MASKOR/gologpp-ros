cmake_minimum_required(VERSION 3.0)
project(gologpp_agent)


add_definitions(-DSOURCE_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}\")
find_package(PkgConfig)

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
)

find_package(catkin OPTIONAL_COMPONENTS
	actionlib
	darknet_action_msgs
	naoqi_wrapper_msgs
	naoqi_bridge_msgs
	move_base_msgs
	opencv_apps_action_msgs
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
	src/action_manager.cpp
	src/exog_manager.cpp
)

#add_dependencies(naoqi_say_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
if(naoqi_wrapper_msgs_FOUND)
	target_sources(gologpp_agent PRIVATE src/naoqi_wrapper_actions.cpp)
	target_compile_definitions(gologpp_agent PUBLIC -DNAOQI_WRAPPER_MSGS_PKG)
endif()

if(naoqi_bridge_msgs_FOUND)
	target_sources(gologpp_agent PRIVATE src/naoqi_bridge_actions.cpp)
	target_compile_definitions(gologpp_agent PUBLIC -DNAOQI_BRIDGE_MSGS_PKG)
endif()


if(darknet_action_msgs_FOUND)
	target_sources(gologpp_agent PRIVATE src/darknet_actions.cpp)
	target_compile_definitions(gologpp_agent PUBLIC -DDARKNET_ACTION_MSGS_PKG)
endif()

if(move_base_msgs_FOUND)
	target_sources(gologpp_agent PRIVATE src/move_base_actions.cpp)
	target_compile_definitions(gologpp_agent PUBLIC -DMOVE_BASE_MSGS_PKG)
endif()

if(opencv_apps_action_msgs_FOUND)
	target_sources(gologpp_agent PRIVATE src/opencv_apps_actions.cpp)
	target_compile_definitions(gologpp_agent PUBLIC -DOPENCV_APPS_MSGS_PKG)
endif()

target_link_libraries(
    gologpp_agent
    ${catkin_LIBRARIES}
    ${readylogpp_LIBRARIES}
    ${parsegologpp_LIBRARIES}
)

target_compile_features(
	gologpp_agent
	PUBLIC
	cxx_std_17
)
