#!/usr/bin/env python

# Copyright (C) 2017 Udacity Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Author: Harsh Pandya

# import modules
import rospy
import tf
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpmath import *
from sympy import *
import numpy as np


def euler2R(roll_x, pitch_y, yaw_z):
	R_yaw_z = Matrix([[np.cos(yaw_z), -np.sin(yaw_z), 0],
					  [np.sin(yaw_z),  np.cos(yaw_z), 0],
					  [0            ,  0            , 1]])
	R_pitch_y = Matrix([[np.cos(pitch_y), 0, np.sin(pitch_y)],
						[0              , 1, 0],
						[-np.sin(pitch_y), 0, np.cos(pitch_y)]])
	R_roll_x = Matrix([[1, 0, 0],
					  [0, np.cos(roll_x), -np.sin(roll_x)],
					  [0, np.sin(roll_x), np.cos(roll_x)]])
	
	return R_yaw_z * R_pitch_y * R_roll_x



def handle_calculate_IK(req):
	rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
	if len(req.poses) < 1:
		print "No valid poses received"
		return -1
	else:
		### Your FK code here
		# Create symbols
		q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')
		alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')
		a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
		d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8') 
		#
		# Create Modified DH parameters
		s = {alpha0:     0,  a0:      0, d1:  0.75, q1:q1,
			alpha1: -pi/2,  a1:   0.35, d2:     0, q2:q2-pi/2,
			alpha2:     0,  a2:   1.25, d3:     0, q3:q3,
			alpha3: -pi/2,  a3: -0.054, d4:   1.5, q4:q4,
			alpha4:  pi/2,  a4:      0, d5:     0, q5:q5,
			alpha5: -pi/2,  a5:      0, d6:     0, q6:q6,
			alpha6:     0,  a6:      0, d7: 0.303, q7:0}

		# Define Modified DH Transformation matrix
		def DH_transformation_matrix(alpha, a, d, q):
			T = Matrix([[           cos(q),           -sin(q),           0,             a],
						[sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d],
						[sin(q)*sin(alpha), cos(q)*sin(alpha),  cos(alpha),  cos(alpha)*d],
						[                0,                 0,           0,             1]])
			return T

		# Create individual transformation matrices
		T0_1 = DH_transformation_matrix(s[alpha0], s[a0], s[d1], s[q1])
		T1_2 = DH_transformation_matrix(s[alpha1], s[a1], s[d2], s[q2])
		T2_3 = DH_transformation_matrix(s[alpha2], s[a2], s[d3], s[q3])
		T3_4 = DH_transformation_matrix(s[alpha3], s[a3], s[d4], s[q4])
		T4_5 = DH_transformation_matrix(s[alpha4], s[a4], s[d5], s[q5])
		T5_6 = DH_transformation_matrix(s[alpha5], s[a5], s[d6], s[q6])
		T6_EE = DH_transformation_matrix(s[alpha6], s[a6], s[d7], s[q7])
		
		# Extract rotation matrices from the transformation matrices
		# R0_EE = T0_EE[:3,:3]
		#
		###

		# Initialize service response
		joint_trajectory_list = []
		for x in xrange(0, len(req.poses)):
			# IK code starts here
			joint_trajectory_point = JointTrajectoryPoint()

		# Extract end-effector position and orientation from request
		# px,py,pz = end-effector position
		# roll, pitch, yaw = end-effector orientation
			px = req.poses[x].position.x
			py = req.poses[x].position.y
			pz = req.poses[x].position.z

			(roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
				[req.poses[x].orientation.x, req.poses[x].orientation.y,
					req.poses[x].orientation.z, req.poses[x].orientation.w])

			### Your IK code here
			# Compensate for rotation discrepancy between DH parameters and Gazebo
			R = euler2R(roll, pitch, yaw)
			R_corr = Matrix([[0, 0, 1],
							 [0,-1, 0],
							 [1, 0, 0]])

			# correct the rotation of end-effector
			R_ee = R * R_corr

			d = 0.303 #distance between joint 6 frame and end-effector frame
			wrist = [px - d * R_ee[0,2],
					 py - d * R_ee[1,2],
					 pz - d * R_ee[2,2]]

			# side lengths of the triangle formed by joint 2, joint 3, and wrist center
			A = 1.501
			B = sqrt((sqrt(wrist[0]**2 + wrist[1]**2) - 0.35)**2 + (wrist[2]-0.75)**2)
			C = 1.25

			# using Law of Cosine to compute the angles inside the triangle (only 2 are needed)
			a = acos((B*B + C*C - A*A)/(2*B*C))
			b = acos((A*A + C*C - B*B)/(2*A*C))

			# calculation of first three joint angles, details can be found in README.md
			theta1 = (atan2(wrist[1],wrist[0]))
			theta2 = (np.pi/2 - a - atan2(wrist[2]-0.75, sqrt(wrist[0]**2 + wrist[1]**2) - 0.35) )
			theta3 = (np.pi/2 - (b + 0.036))

			# calculate the numerical representation of R3_6 given theta1, theta2, theta3
			# use the factor that rotation matrix inverse is the transpose
			R0_3 = (T0_1[0:3,0:3] * T1_2[0:3,0:3] * T2_3[0:3,0:3]).evalf(subs={q1:theta1, q2:theta2, q3:theta3})
			R3_6 = R0_3.T * R_ee

			# calculate the remaining joint angle, atan2 is used for angle disambiguity
			# We will consider two cases:
			# 1. theta5 is singular, meaning theta5 = 0
			# R3_6[0,2] = R3_6[2,2] = R3_6[1,0] = R3_6[1,1] = 0 and R3_6[1,2] = 1
			# R3_6[1,2] is used to determine if theta5 = 0 because that element only depends on theta5
			if abs(1-R3_6[1,2]) < 1e-2:
				theta5 = 0
				
				# theta4 = theta6 is chosen for convenience
				theta4 = atan2(-R3_6[2,0], R3_6[0,0]) / 2
				theta6 = theta4

			# 2. theta5 != 0
			else:
				x4 = -R3_6[0,2]
				y4 = R3_6[2,2]
				theta4 = (atan2(y4, x4))

				x5 = R3_6[1,2]
				y5 = sqrt(R3_6[0,2]**2 + R3_6[2,2]**2)
				theta5 = (atan2(y5, x5))

				x6 = R3_6[1,0]
				y6 = -R3_6[1,1] 
				theta6 = (atan2(y6, x6))

			# Populate response for the IK request
			# In the next line replace theta1,theta2...,theta6 by your joint angle variables
			joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
			print joint_trajectory_point.positions
			joint_trajectory_list.append(joint_trajectory_point)

		rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
		return CalculateIKResponse(joint_trajectory_list)


def IK_server():
	# initialize node and declare calculate_ik service
	rospy.init_node('IK_server')
	s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
	print "Ready to receive an IK request"
	rospy.spin()

if __name__ == "__main__":
	IK_server()
