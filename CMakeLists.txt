cmake_minimum_required(VERSION 2.8.3)

project(openhri_ros)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  audio_common_msgs
)


###
# catkin specific configuration
catkin_package(
  CATKIN_DEPENDS rospy std_msgs audio_common_msgs
)

catkin_python_setup()

###
# Install

#install(TARGETS python_module_library
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
#)

catkin_install_python(PROGRAMS
  scripts/GoogleSpeechRecog.py
  scripts/GoogleTextToSpeech.py
  scripts/JuliusRos.py
  scripts/OpenJTalk.py

  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY etc/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/etc
)
