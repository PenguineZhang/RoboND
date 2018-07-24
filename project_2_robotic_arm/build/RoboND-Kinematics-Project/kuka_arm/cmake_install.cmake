# Install script for directory: /home/peng/RoboND/Term_1/project_2_robotic_arm/src/RoboND-Kinematics-Project/kuka_arm

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/peng/RoboND/Term_1/project_2_robotic_arm/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/kuka_arm/srv" TYPE FILE FILES "/home/peng/RoboND/Term_1/project_2_robotic_arm/src/RoboND-Kinematics-Project/kuka_arm/srv/CalculateIK.srv")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/kuka_arm/cmake" TYPE FILE FILES "/home/peng/RoboND/Term_1/project_2_robotic_arm/build/RoboND-Kinematics-Project/kuka_arm/catkin_generated/installspace/kuka_arm-msg-paths.cmake")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/peng/RoboND/Term_1/project_2_robotic_arm/devel/include/kuka_arm")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/peng/RoboND/Term_1/project_2_robotic_arm/devel/share/roseus/ros/kuka_arm")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/peng/RoboND/Term_1/project_2_robotic_arm/devel/share/common-lisp/ros/kuka_arm")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/peng/RoboND/Term_1/project_2_robotic_arm/devel/share/gennodejs/ros/kuka_arm")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  execute_process(COMMAND "/usr/bin/python" -m compileall "/home/peng/RoboND/Term_1/project_2_robotic_arm/devel/lib/python2.7/dist-packages/kuka_arm")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages" TYPE DIRECTORY FILES "/home/peng/RoboND/Term_1/project_2_robotic_arm/devel/lib/python2.7/dist-packages/kuka_arm")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/peng/RoboND/Term_1/project_2_robotic_arm/build/RoboND-Kinematics-Project/kuka_arm/catkin_generated/installspace/kuka_arm.pc")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/kuka_arm/cmake" TYPE FILE FILES "/home/peng/RoboND/Term_1/project_2_robotic_arm/build/RoboND-Kinematics-Project/kuka_arm/catkin_generated/installspace/kuka_arm-msg-extras.cmake")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/kuka_arm/cmake" TYPE FILE FILES
    "/home/peng/RoboND/Term_1/project_2_robotic_arm/build/RoboND-Kinematics-Project/kuka_arm/catkin_generated/installspace/kuka_armConfig.cmake"
    "/home/peng/RoboND/Term_1/project_2_robotic_arm/build/RoboND-Kinematics-Project/kuka_arm/catkin_generated/installspace/kuka_armConfig-version.cmake"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/kuka_arm" TYPE FILE FILES "/home/peng/RoboND/Term_1/project_2_robotic_arm/src/RoboND-Kinematics-Project/kuka_arm/package.xml")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/kuka_arm" TYPE PROGRAM FILES
    "/home/peng/RoboND/Term_1/project_2_robotic_arm/src/RoboND-Kinematics-Project/kuka_arm/scripts/random_spawn.py"
    "/home/peng/RoboND/Term_1/project_2_robotic_arm/src/RoboND-Kinematics-Project/kuka_arm/scripts/safe_spawner.sh"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/kuka_arm" TYPE FILE FILES "/home/peng/RoboND/Term_1/project_2_robotic_arm/src/RoboND-Kinematics-Project/kuka_arm/config/spawn_locations.yaml")
endif()

