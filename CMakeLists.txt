cmake_minimum_required(VERSION 3.5)
project(gologpp_agent)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_definitions(-DSOURCE_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}\")
# suppress all warnings
add_definitions(-w)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)

set(gologpp_action_pkgs
	turtlesim
)

find_package(${gologpp_action_pkgs})

if(CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated-declarations -Wno-placement-new")
endif()


set(BUILD_TESTS OFF CACHE BOOL "Enable gologpp tests" FORCE)

add_executable(gologpp_agent
	src/agent_node.cpp
	src/ros_backend.cpp
	src/action_manager.cpp
	src/exog_manager.cpp
)
ament_target_dependencies(gologpp_agent rclcpp rclcpp_action rclcpp_components turtlesim std_msgs)
target_include_directories(gologpp_agent PUBLIC
	src/
	include/golog++
	include/eclipse
)

install(
	TARGETS gologpp_agent
	DESTINATION lib/${PROJECT_NAME}
)

foreach(pkg ${gologpp_action_pkgs})
	if ("${${pkg}_FOUND}")
		target_sources(gologpp_agent PRIVATE "src/actions/gologpp_${pkg}.cpp")
		target_compile_definitions(gologpp_agent PUBLIC "-D${pkg}_FOUND")
	endif()
endforeach()

target_link_libraries(
	gologpp_agent
	${CMAKE_CURRENT_SOURCE_DIR}/lib/libeclipse.so
	${CMAKE_CURRENT_SOURCE_DIR}/lib/libgolog++.so.0.1.0
	${CMAKE_CURRENT_SOURCE_DIR}/lib/libtaptenc-golog++.so.0.1.0
	${CMAKE_CURRENT_SOURCE_DIR}/lib/libreadylog++.so.0.1.0
	${CMAKE_CURRENT_SOURCE_DIR}/lib/libparsegolog++.so.0.1.0
)

install(
  DIRECTORY lib/
  DESTINATION lib
)

target_compile_features(
	gologpp_agent
	PUBLIC
	c_std_99
	cxx_std_17
)
ament_package()
