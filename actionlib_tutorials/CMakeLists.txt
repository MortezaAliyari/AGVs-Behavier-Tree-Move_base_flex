cmake_minimum_required(VERSION 2.8.3)
project(actionlib_tutorials)

find_package(catkin REQUIRED COMPONENTS roscpp actionlib actionlib_msgs)
find_package(Boost REQUIRED COMPONENTS system)

add_action_files(
  DIRECTORY action
  FILES Robotmode.action 
)

generate_messages(
  DEPENDENCIES actionlib_msgs std_msgs
)

catkin_package(
  CATKIN_DEPENDS actionlib_msgs

)

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

add_executable(rtmode_server src/rtmode_server.cpp)

target_link_libraries(
  rtmode_server
  ${catkin_LIBRARIES}
)

add_dependencies(
  rtmode_server
  ${actionlib_tutorials_EXPORTED_TARGETS}
)
add_executable(rtmode_client src/rtmode_client.cpp)

target_link_libraries( 
  rtmode_client
  ${catkin_LIBRARIES}
)

add_dependencies(
  rtmode_client
  ${actionlib_tutorials_EXPORTED_TARGETS}
)

add_executable(thread_test src/thread_test.cpp)

target_link_libraries(
  thread_test
  ${catkin_LIBRARIES}
)

add_dependencies(
  thread_test
  ${actionlib_tutorials_EXPORTED_TARGETS}
)

