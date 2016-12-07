cmake_minimum_required(VERSION 2.8.3)
project(example_ros_cpp)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)



## Generate messages in the 'msg' folder
# add_message_files(FILES
#   Message1.msg
# )

## Generate services in the 'srv' folder
# add_service_files(FILES
#   Service1.srv
# )


## Generate added messages and services with any dependencies listed here
# generate_messages(DEPENDENCIES
#   std_msgs  # or other packages containing msgs
# )

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs
)

include_directories(
  include ${catkin_INCLUDE_DIRS}
)


## Declare a C++ executable
add_executable(pid_controller_node src/pid_controller_base.cpp src/pid_controller_ros.cpp src/nodes/pid_controller_node.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(example_ros_cpp_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(pid_controller_node
  ${catkin_LIBRARIES}
)
