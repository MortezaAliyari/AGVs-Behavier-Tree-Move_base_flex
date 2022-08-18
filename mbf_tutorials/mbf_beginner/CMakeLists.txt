cmake_minimum_required(VERSION 2.8.3)
project(mbf_beginner)

find_package(catkin REQUIRED)

###################################
## catkin specific configuration ##
###################################

catkin_package()

catkin_install_python(
  PROGRAMS  scripts/mb_relay.py 
            scripts/mb_relay_client.py 
            scripts/mb_relay_server.py 
            scripts/mb_relay_subscriber.py 
            scripts/mbf_goal_client.py
            scripts/mbf_path_client.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
