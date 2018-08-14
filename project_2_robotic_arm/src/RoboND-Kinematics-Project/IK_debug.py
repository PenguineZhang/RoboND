from sympy import *
from time import time
from mpmath import radians
import numpy as np
import tf

'''
Format of test case is [ [[EE position],[EE orientation as quaternions]],[WC location],[joint angles]]
You can generate additional test cases by setting up your kuka project and running `$ roslaunch kuka_arm forward_kinematics.launch`
From here you can adjust the joint angles to find thetas, use the gripper to extract positions and orientation (in quaternion xyzw) and lastly use link 5
to find the position of the wrist center. These newly generated test cases can be added to the test_cases dictionary.
'''

test_cases = {1:[[[2.16135,-1.42635,1.55109],
                  [0.708611,0.186356,-0.157931,0.661967]],
                  [1.89451,-1.44302,1.69366],
                  [-0.65,0.45,-0.36,0.95,0.79,0.49]],
              2:[[[-0.56754,0.93663,3.0038],
                  [0.62073, 0.48318,0.38759,0.480629]],
                  [-0.638,0.64198,2.9988],
                  [-0.79,-0.11,-2.33,1.94,1.14,-3.68]],
              3:[[[-1.3863,0.02074,0.90986],
                  [0.01735,-0.2179,0.9025,0.371016]],
                  [-1.1669,-0.17989,0.85137],
                  [-2.99,-0.12,0.94,4.06,1.29,-4.12]],
              4:[],
              5:[]}
def clip_theta1(theta):
    low_ = np.deg2rad(-185)
    high_ = np.deg2rad(185)
    return np.clip(theta, low_, high_)

def clip_theta2(theta):
    low_ = np.deg2rad(-45)
    high_ = np.deg2rad(85)
    return np.clip(theta, low_, high_)

def clip_theta3(theta):
    low_ = np.deg2rad(-210)
    high_ = np.deg2rad(155)
    return np.clip(theta, low_, high_)

def clip_theta4(theta):
    low_ = np.deg2rad(-350)
    high_ = np.deg2rad(350)
    return np.clip(theta, low_, high_)

def clip_theta5(theta):
    low_ = np.deg2rad(-125)
    high_ = np.deg2rad(125)
    return np.clip(theta, low_, high_)

def clip_theta6(theta):
    low_ = np.deg2rad(-350)
    high_ = np.deg2rad(350)
    return np.clip(theta, low_, high_)

def qua2euler(q):
    # q[0]:x, q[1]:y, q[2]: z, q[3]:w
    R = np.array( [[1-2*(q[1]**2+q[2]**2), 2*(q[0]*q[1]-q[3]*q[2]), 2*(q[3]*q[1]+q[0]*q[2])],
                   [2*(q[0]*q[1]+q[3]*q[2]), 1-2*(q[0]**2+q[2]**2), 2*(q[1]*q[2]-q[3]*q[0])],
                   [2*(q[0]*q[2]-q[3]*q[1]), 2*(q[3]*q[0]+q[1]*q[2]), 1-2*(q[0]**2+q[1]**2)]])
    return R

def euler2R(roll_x, pitch_y, yaw_z):
    R_yaw_z = Matrix([[np.cos(yaw_z), -np.sin(yaw_z), 0],
                     [np.sin(yaw_z),  np.cos(yaw_z), 0],
                     [0          ,  0          , 1]])
    R_pitch_y = Matrix([[np.cos(pitch_y), 0, np.sin(pitch_y)],
                        [0, 1, 0],
                        [-np.sin(pitch_y), 0, np.cos(pitch_y)]])
    R_roll_x = Matrix([[1, 0, 0],
                      [0, np.cos(roll_x), -np.sin(roll_x)],
                      [0, np.sin(roll_x), np.cos(roll_x)]])
    
    return R_yaw_z * R_pitch_y * R_roll_x

def DH_transformation_matrix(alpha, a, d, q):
    T = Matrix([[           cos(q),           -sin(q),           0,             a],
                [sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d],
                [sin(q)*sin(alpha), cos(q)*sin(alpha),  cos(alpha),  cos(alpha)*d],
                [                0,                 0,           0,             1]])
    return T

def test_code(test_case):
    ## Set up code
    ## Do not modify!
    x = 0
    class Position:
        def __init__(self,EE_pos):
            self.x = EE_pos[0]
            self.y = EE_pos[1]
            self.z = EE_pos[2]
    class Orientation:
        def __init__(self,EE_ori):
            self.x = EE_ori[0]
            self.y = EE_ori[1]
            self.z = EE_ori[2]
            self.w = EE_ori[3]

    position = Position(test_case[0][0])
    orientation = Orientation(test_case[0][1])

    class Combine:
        def __init__(self,position,orientation):
            self.position = position
            self.orientation = orientation

    comb = Combine(position,orientation)

    class Pose:
        def __init__(self,comb):
            self.poses = [comb]

    req = Pose(comb)
    start_time = time()

    ########################################################################################
    ## 

    ## Insert IK code here!
    q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')
    alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')
    a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
    d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8') 

    s = {alpha0:     0,  a0:      0, d1:  0.75, q1:q1,
         alpha1: -pi/2,  a1:   0.35, d2:     0, q2:q2-pi/2,
         alpha2:     0,  a2:   1.25, d3:     0, q3:q3,
         alpha3: -pi/2,  a3: -0.054, d4:   1.5, q4:q4,
         alpha4:  pi/2,  a4:      0, d5:     0, q5:q5,
         alpha5: -pi/2,  a5:      0, d6:     0, q6:q6,
         alpha6:     0,  a6:      0, d7: 0.303, q7:0}

    T0_1 = DH_transformation_matrix(s[alpha0], s[a0], s[d1], s[q1])
    T1_2 = DH_transformation_matrix(s[alpha1], s[a1], s[d2], s[q2])
    T2_3 = DH_transformation_matrix(s[alpha2], s[a2], s[d3], s[q3])
    T3_4 = DH_transformation_matrix(s[alpha3], s[a3], s[d4], s[q4])
    T4_5 = DH_transformation_matrix(s[alpha4], s[a4], s[d5], s[q5])
    T5_6 = DH_transformation_matrix(s[alpha5], s[a5], s[d6], s[q6])
    T6_EE = DH_transformation_matrix(s[alpha6], s[a6], s[d7], s[q7])

    quaternion = (req.poses[0].orientation.x,
                  req.poses[0].orientation.y,
                  req.poses[0].orientation.z,
                  req.poses[0].orientation.w)

    euler = tf.transformations.euler_from_quaternion(quaternion)

    R = euler2R(euler[0], euler[1], euler[2])
    R_corr = Matrix([[0, 0, 1],
                     [0,-1, 0],
                     [1, 0, 0]])

    R_ee = R * R_corr
    d = 0.303
    wrist = [req.poses[0].position.x - d * R_ee[0,2],
             req.poses[0].position.y - d * R_ee[1,2],
             req.poses[0].position.z - d * R_ee[2,2]]

    A = 1.501
    B = sqrt((sqrt(wrist[0]**2 + wrist[1]**2) - 0.35)**2 + (wrist[2]-0.75)**2)
    C = 1.25

    a = acos((B*B + C*C - A*A)/(2*B*C))
    b = acos((A*A + C*C - B*B)/(2*A*C))

    theta1 = (atan2(wrist[1],wrist[0]))
    theta2 = np.pi/2 - a - atan2(wrist[2]-0.75, sqrt(wrist[0]**2 + wrist[1]**2)-0.35)
    theta3 = np.pi/2 - (b + 0.036)

    R0_3 = ((T0_1 * T1_2 * T2_3)[:3,:3]).evalf(subs={q1:theta1, q2:theta2, q3:theta3})
    R3_6 = R0_3.inv(method='LU') * R_ee


    theta4 = atan2(R3_6[2,2], -R3_6[0,2])
    theta5 = atan2(sqrt(R3_6[0,2]**2 + R3_6[2,2]**2), R3_6[1,2])
    theta6 = atan2(-R3_6[1,1], R3_6[1,0])
    print R3_6[:,2]
    # print theta1, theta2, theta3, theta4, theta5, theta6
    ## 
    ########################################################################################
    
    ########################################################################################
    ## For additional debugging add your forward kinematics here. Use your previously calculated thetas
    ## as the input and output the position of your end effector as your_ee = [x,y,z]
    
    ## (OPTIONAL) YOUR CODE HERE!

    T0_EE = (T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_EE).evalf(subs={q1:theta1,q2:theta2,q3:theta3, q4:theta4, q5:theta5, q6:theta6})
    ## End your code input for forward kinematics here!
    ########################################################################################

    ## For error analysis please set the following variables of your WC location and EE location in the format of [x,y,z]
    your_wc = [wrist[0],wrist[1],wrist[2]] # <--- Load your calculated WC values in this array
    your_ee = [T0_EE[0,3],T0_EE[1,3],T0_EE[2,3]] # <--- Load your calculated end effector value from your forward kinematics
    # your_ee = [1,1,1]
    ########################################################################################

    ## Error analysis
    print ("\nTotal run time to calculate joint angles from pose is %04.4f seconds" % (time()-start_time))

    # Find WC error
    if not(sum(your_wc)==3):
        wc_x_e = abs(your_wc[0]-test_case[1][0])
        wc_y_e = abs(your_wc[1]-test_case[1][1])
        wc_z_e = abs(your_wc[2]-test_case[1][2])
        wc_offset = sqrt(wc_x_e**2 + wc_y_e**2 + wc_z_e**2)
        print ("\nWrist error for x position is: %04.8f" % wc_x_e)
        print ("Wrist error for y position is: %04.8f" % wc_y_e)
        print ("Wrist error for z position is: %04.8f" % wc_z_e)
        print ("Overall wrist offset is: %04.8f units" % wc_offset)

    # Find theta errors
    t_1_e = abs(theta1-test_case[2][0])
    t_2_e = abs(theta2-test_case[2][1])
    t_3_e = abs(theta3-test_case[2][2])
    t_4_e = abs(theta4-test_case[2][3])
    t_5_e = abs(theta5-test_case[2][4])
    t_6_e = abs(theta6-test_case[2][5])
    print ("\nTheta 1 error is: %04.8f" % t_1_e)
    print ("Theta 2 error is: %04.8f" % t_2_e)
    print ("Theta 3 error is: %04.8f" % t_3_e)
    print ("Theta 4 error is: %04.8f" % t_4_e)
    print ("Theta 5 error is: %04.8f" % t_5_e)
    print ("Theta 6 error is: %04.8f" % t_6_e)
    print ("\n**These theta errors may not be a correct representation of your code, due to the fact \
           \nthat the arm can have muliple positions. It is best to add your forward kinmeatics to \
           \nconfirm whether your code is working or not**")
    print (" ")

    # Find FK EE error
    if not(sum(your_ee)==3):
        ee_x_e = abs(your_ee[0]-test_case[0][0][0])
        ee_y_e = abs(your_ee[1]-test_case[0][0][1])
        ee_z_e = abs(your_ee[2]-test_case[0][0][2])
        ee_offset = sqrt(ee_x_e**2 + ee_y_e**2 + ee_z_e**2)
        print ("\nEnd effector error for x position is: %04.8f" % ee_x_e)
        print ("End effector error for y position is: %04.8f" % ee_y_e)
        print ("End effector error for z position is: %04.8f" % ee_z_e)
        print ("Overall end effector offset is: %04.8f units \n" % ee_offset)




if __name__ == "__main__":
    # Change test case number for different scenarios
    test_case_number = 1

    test_code(test_cases[test_case_number])
