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
	rosconsole
	std_msgs
	genmsg
	message_generation
	actionlib_msgs
	actionlib
)

set(gologpp_action_pkgs
	darknet_action_msgs
	naoqi_wrapper_msgs
	naoqi_bridge_msgs
	move_base_msgs
	opencv_apps_action_msgs
)

find_package(catkin OPTIONAL_COMPONENTS ${gologpp_action_pkgs})

add_definitions(-DLIBQI_VERSION=${naoqi_libqi_VERSION_MAJOR}${naoqi_libqi_VERSION_MINOR})
if(CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated-declarations -Wno-placement-new")
endif()


catkin_package(
	#INCLUDE_DIRS include
	#LIBRARIES ${PROJECT_NAME}
	CATKIN_DEPENDS actionlib_msgs actionlib roscpp
	${gologpp_action_pkgs}
)

set(BUILD_TESTS OFF CACHE BOOL "Enable gologpp tests" FORCE)

link_directories(${readylogpp_LIBRARY_DIRS} ${parsegologpp_LIBRARY_DIRS})

add_executable(gologpp_agent
	src/agent_node.cpp
	src/ros_backend.cpp
	src/action_manager.cpp
	src/exog_manager.cpp
)

target_include_directories(gologpp_agent PUBLIC
	src/
	${catkin_INCLUDE_DIRS}
	${readylogpp_INCLUDE_DIRS}
	${parsegologpp_INCLUDE_DIRS}
)

foreach(pkg ${gologpp_action_pkgs})
	if ("${${pkg}_FOUND}")
		target_sources(gologpp_agent PRIVATE "src/actions/gologpp_${pkg}.cpp")
		target_compile_definitions(gologpp_agent PUBLIC "-D${pkg}_FOUND")
	endif()
endforeach()

target_link_libraries(
	gologpp_agent
	roscpp
	rosconsole
	actionlib
	${catkin_LIBRARIES}
	${readylogpp_LIBRARIES}
	${parsegologpp_LIBRARIES}
)

target_compile_features(
	gologpp_agent
	PUBLIC
	cxx_std_17
)
