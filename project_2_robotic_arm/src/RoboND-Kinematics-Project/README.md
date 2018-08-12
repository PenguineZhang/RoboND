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
[wc]: ./img/wc_cal.png
[wc_math]: ./img/wc_cal_math.png

## Project Procedure
### Obtain DH parameter table
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

From joint<sub>i-1</sub> to joint<sub>i</sub>, the transform matrix (rotation + translation) is the following:
![mat]

In this project, the homogeneous transform between each joint in shown below with constants substituted:

```
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

T6_EE = Matrix([
		[1, 0, 0,     0],
		[0, 1, 0,     0],
		[0, 0, 1, 0.303],
		[0, 0, 0,     1]])
```

---
### Kinematic Analysis
#### Forward Kinematics
Forward kinematics (FK) is to use the known joint angles to drive each joint towards a desired end-effector location. To do that, one can simply substitute the joint angles into `T0_1`, `T1_2`, etc and multiply the matrices,
```
T0_EE = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_EE
```
The above equation calculates the transform between the base_link and end_effect.

#### Inverse Kinematics
Inverse kinematics (IK) is doing the opposite of FK. In the IK process, joint angles are calculated given the end-effector location. To solve the each joint angle, decoupling technique has been utilized to divide the problme into Inverse Position Kinematics and Inverse Orientation Kinematics, which are explained below.


##### Inverse Position Kinematics
The last three joints compose the spherical joints, where rotation axes intersect at a single point called wrist center. The wrist center determines the position of the end-effect. The way to calculate the wrist center is to back-trace along the end-effector orientation:

![wc]

To compute the wrist center analytically, one can use equation:

![wc_math]

where <sup>0</sup><sub>6</sub>R

##### Inverse Orientation Kinematics

And here's where you can draw out and show your math for the derivation of your theta angles. 

![alt text][image2]

### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 


Here I'll talk about the code, what techniques I used, what worked and why, where the implementation might fail and how I might improve it if I were going to pursue this project further.  


And just for fun, another example image:
![alt text][image3]


