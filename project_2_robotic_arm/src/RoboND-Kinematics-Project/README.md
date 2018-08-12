## Project: Kinematics Pick & Place
### Project Objective
This project focuses on controlling kuka arm to pick up target and drop it into the bin in a simulation environment. The objective is to compute the forward and inverse kinematic of the arm by calculating each joint angles and cooresponding frame transformation.

---

[//]: # (Image References)

[dh]: ./img/DH_param.png
[motion]: ./img/resulting_motion.gif
[kuka]: ./img/kuka_arm.png
[result]: ./img/task_completion.jpg
[sketch]: ./img/kinematic_analysis.png
[mat]: ./img/matrix.png

## Project Procedure
### Obtaining DH parameter table
Below is the simplified representation of the kuka arm with partial DH parameters labelled.
![kuka]

With given labelled frame at each joint and the data from kr210.urdf.xacro file, one can derive the following DH parameter table.

Links | alpha(i-1) | a(i-1) | d(i) | theta(i)
--- | --- | --- | --- | ---
0->1 | 0 | 0 | 0.75 | q<sub>1</sub>
1->2 | -pi/2 | 0.35 | 0 | -pi/2 + q<sub>2</sub>
2->3 | 0 | 1.25 | 0 | q<sub>3</sub>
3->4 |  -pi/2 | -0.054 | 1.5 | q<sub>4</sub>
4->5 | pi/2 | 0 | 0 | q<sub>5</sub>
5->6 | -pi/2 | 0 | 0 | q<sub>6</sub>
6->EE | 0 | 0 | 0.303 | 0

Note that since there is a -90 degrees rotation offset from joint1 to joint2, subtraction of pi/2 has been added in forward kinematic calculation.

From joint<sub>i-1</sub> to joint_<sub>i</sub>, the transform matrix (rotation + translation) is the following:
![matrix.png]

In this project, the transform between each joint in shown below with constants substituted:

'''
T0_1 = 	Matrix([
		[cos(q1), -sin(q1), 0,    0],
		[sin(q1),  cos(q1), 0,    0],
		[      0,        0, 1, 0.75],
		[      0,        0, 0,    1]])

T1_2 = 	Matrix([
		[sin(q2),  cos(q2), 0, 0.35],
		[      0,        0, 1,    0],
		[cos(q2), -sin(q2), 0,    0],
		[      0,        0, 0,    1]])

T2_3 = 	Matrix([
		[cos(q1), -sin(q1), 0,    0],
		[sin(q1),  cos(q1), 0,    0],
		[      0,        0, 1, 0.75],
		[      0,        0, 0,    1]])

T3_4 = 	Matrix([
		[ cos(q4), -sin(q4), 0, -0.054],
		[       0,        0, 1,    1.5],
		[-sin(q4), -cos(q4), 0,      0],
		[       0,        0, 0,      1]])

T4_5 = 	Matrix([
		[cos(q5), -sin(q5),  0, 0],
		[      0,        0, -1, 0],
		[sin(q5),  cos(q5),  0, 0],
		[      0,        0,  0, 1]])

T5_6 = Matrix([
		[ cos(q6), -sin(q6), 0, 0],
		[       0,        0, 1, 0],
		[-sin(q6), -cos(q6), 0, 0],
		[       0,        0, 0, 1]])
'''

---
### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

Here is an example of how to include an image in your writeup.

#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.




#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

And here's where you can draw out and show your math for the derivation of your theta angles. 

![alt text][image2]

### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 


Here I'll talk about the code, what techniques I used, what worked and why, where the implementation might fail and how I might improve it if I were going to pursue this project further.  


And just for fun, another example image:
![alt text][image3]


