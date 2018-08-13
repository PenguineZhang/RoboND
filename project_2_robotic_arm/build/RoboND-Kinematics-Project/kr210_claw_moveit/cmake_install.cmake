# Install script for directory: /home/peng/RoboND/Term1/project_2_robotic_arm/src/RoboND-Kinematics-Project/kr210_claw_moveit

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/peng/RoboND/Term1/project_2_robotic_arm/install")
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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/peng/RoboND/Term1/project_2_robotic_arm/build/RoboND-Kinematics-Project/kr210_claw_moveit/catkin_generated/installspace/kr210_claw_moveit.pc")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/kr210_claw_moveit/cmake" TYPE FILE FILES
    "/home/peng/RoboND/Term1/project_2_robotic_arm/build/RoboND-Kinematics-Project/kr210_claw_moveit/catkin_generated/installspace/kr210_claw_moveitConfig.cmake"
    "/home/peng/RoboND/Term1/project_2_robotic_arm/build/RoboND-Kinematics-Project/kr210_claw_moveit/catkin_generated/installspace/kr210_claw_moveitConfig-version.cmake"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/kr210_claw_moveit" TYPE FILE FILES "/home/peng/RoboND/Term1/project_2_robotic_arm/src/RoboND-Kinematics-Project/kr210_claw_moveit/package.xml")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/kr210_claw_moveit" TYPE DIRECTORY FILES "/home/peng/RoboND/Term1/project_2_robotic_arm/src/RoboND-Kinematics-Project/kr210_claw_moveit/launch" REGEX "/setup\\_assistant\\.launch$" EXCLUDE)
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/kr210_claw_moveit" TYPE DIRECTORY FILES "/home/peng/RoboND/Term1/project_2_robotic_arm/src/RoboND-Kinematics-Project/kr210_claw_moveit/config")
endif()

