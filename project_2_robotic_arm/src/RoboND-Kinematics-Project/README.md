## Project: Kinematics Pick & Place
### Project Objective
This project focuses on controlling kuka arm to pick up target and drop it into the bin in a simulation environment. The objective is to compute the forward and inverse kinematic of the arm by calculating each joint angles and cooresponding frame transformation.

---

[//]: # (Image References)

[dh]: ./img/DH_param.png
[motion]: ./img/resulting_motion.gif
[kuka]: ./img/kuka_arm.png
[result]: ./img/task_completion

## Project Procedure
### Obtaining DH parameter table
Below is the simplified representation of the kuka arm with partial DH parameters labelled.
![kuka]

---
### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

Here is an example of how to include an image in your writeup.

#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

Links | alpha(i-1) | a(i-1) | d(i) | theta(i)
--- | --- | --- | --- | ---
0->1 | 0 | 0 | 0.75 | q<sub>1</sub>
1->2 | -pi/2 | 0.35 | 0 | -pi/2 + q<sub>2</sub>
2->3 | 0 | 1.25 | 0 | q<sub>3</sub>
3->4 |  -pi/2 | -0.054 | 1.5 | q<sub>4</sub>
4->5 | pi/2 | 0 | 0 | q<sub>5</sub>
5->6 | -pi/2 | 0 | 0 | q<sub>6</sub>
6->EE | 0 | 0 | 0.303 | q<sub>7</sub>


#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

And here's where you can draw out and show your math for the derivation of your theta angles. 

![alt text][image2]

### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 


Here I'll talk about the code, what techniques I used, what worked and why, where the implementation might fail and how I might improve it if I were going to pursue this project further.  


And just for fun, another example image:
![alt text][image3]


