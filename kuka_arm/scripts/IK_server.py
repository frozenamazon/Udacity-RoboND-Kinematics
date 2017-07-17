#!/usr/bin/env python

# Copyright (C) 2017 Electric Movement Inc.
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
import math

numberOfRuns = 0
# Define Modified DH Transformation matrix
def transformation_matrix(alpha,a,d,q):    
    return Matrix([[        cos(q),        -sin(q),       0,           a],
                   [ sin(q)*cos(alpha),  cos(q)*cos(alpha),  -sin(alpha),  -sin(alpha)*d],
                   [ sin(q)*sin(alpha),  cos(q)*sin(alpha),   cos(alpha),   cos(alpha)*d],
                   [             0,              0,        0,        1]])

def getRRRJointAngles(wx,wy,wz):
    # getting the first three revolute joint, ignoringthe spherical arm

    #link between joint 2 to 3
    l2 = 1.25
    #link between joint 3 to wc
    l3 = sqrt(1.5*1.5 + 0.054*0.054)
    #distance in x axis between base link and joint 2
    dx0_1 = 0.35
    #distance in z axis between base link and joint 2
    dz0_1 = 0.75
    #distance in gripper to wc
    dG_wc = 0.303

    xc = (sqrt(wx**2 + wy**2) - dx0_1)
    zc = (wz - dz0_1)
    q1 = theta1 = atan2(wy,wx)

    # theta 2 and theta3 forms a tringle with the wrist center
    # Following the cosine rule. a2 = b2 + c2 - 2bc*cos(theta)
    r = (xc*xc + zc*zc - l3*l3 - l2*l2)/(2*l3*l2)
    theta3 = atan2(sqrt(1-r*r), r)
    theta3b = atan2(-sqrt(1-r*r), r) 

    # Theta 2 is basically the angle of the triangle minus triangle angle
    theta2 = atan2(xc,zc) - atan2(l3*sin(theta3), l2+l3*cos(theta3))
    theta2b = atan2(xc,zc) - atan2(l3*sin(theta3b), l2+l3*cos(theta3b))
    
    # just making sure that theta are evaluated and considered the both + and -
    # theta3 needs to minus 90degree as the joint starts at that position
    q2 = theta2.evalf()
    q2b = theta2b.evalf()
    q3 = (theta3 - np.pi/2).evalf()
    q3b = (-theta3b + np.pi/2).evalf()
    
    # Check for q2,q3. 
    # q3 should be between -pi and pi/2
    # q2, should only be between -pi to pi
    if(q3 < -np.pi) or (q3 > np.pi/2) or (q2 < -np.pi) or (q2 > np.pi):
        q3 = None
        q2 = None
    
    if(q3b < -np.pi) or (q3b > np.pi/2) or (q2b < -np.pi) or (q2b > np.pi):
        q3b = None
        q2b = None
    
    return [q1,q2,q3,q2b,q3b]

def getRRRSpehericalArmJointAngles(yaw, pitch, roll, theta1, theta2, theta3):


    a0 = 0
    d1 = 0.75
    q1 = theta1
    a1 = 0.35
    d2 = 0
    q2 = theta2
    a2 = 1.25
    d3 = 0
    q3 = theta3

    T0_3 = Matrix([
        [sin(q2 + q3)*cos(q1), cos(q1)*cos(q2 + q3), -sin(q1), (a1 + a2*sin(q2))*cos(q1)],
        [sin(q1)*sin(q2 + q3), sin(q1)*cos(q2 + q3),  cos(q1), (a1 + a2*sin(q2))*sin(q1)],
        [        cos(q2 + q3),        -sin(q2 + q3),        0,           a2*cos(q2) + d1],
        [                   0,                    0,        0,                         1]])

    #implementing the Roll Pitch Yaw intrinsic transformation
    alpha = yaw
    beta = pitch
    gamma = roll
    Rrpy = Matrix([[cos(alpha)*cos(beta), cos(alpha)*sin(beta)*sin(gamma) - sin(alpha)*cos(gamma), cos(alpha)*sin(beta)*cos(gamma) + sin(alpha)*sin(gamma)], 
                   [sin(alpha)*cos(beta), sin(alpha)*sin(beta)*sin(gamma) + cos(alpha)*cos(gamma), sin(alpha)*sin(beta)*cos(gamma) - cos(alpha)*sin(gamma)],
                   [          -sin(beta),                                    cos(beta)*sin(gamma), cos(beta)*cos(gamma)]])

    # Include a correction rotation which is rot_z(pi) * rot_y(-pi/2) to fix the end gripper link axis
    # Correction matrix is transpose as the final Rrpy has been included with this difference in axis gripper to the normal DH
    Rcorr = Matrix([
        [0,  0, 1],
        [0, -1, 0],
        [1,  0, 0]])
    R0_3 = T0_3[:3,:3]


    E3_6 = R0_3.transpose() * Rrpy * Rcorr.transpose()
    # LHS of the equation (R3_6 is equal to the RHS)
    #   R3_6 = (T3_4*T4_5*T5_6)[:3,:3] = E3_6
    #   R3_6 = Matrix([
    #     [-sin(q4)*sin(q6) + cos(q4)*cos(q5)*cos(q6), -sin(q4)*cos(q6) - sin(q6)*cos(q4)*cos(q5), -sin(q5)*cos(q4)],
    #     [                           sin(q5)*cos(q6),                           -sin(q5)*sin(q6),          cos(q5)],
    #     [-sin(q4)*cos(q5)*cos(q6) - sin(q6)*cos(q4),  sin(q4)*sin(q6)*cos(q5) - cos(q4)*cos(q6),  sin(q4)*sin(q5)]])

    theta4 = atan2(E3_6[2,2], -E3_6[0,2]).evalf()
    theta6 = atan2(-E3_6[1,1], E3_6[1,0]).evalf()

    #theta5 is atan2 ( sqrt(sin2(q5)*cos2(q6) + sin(q5)*sin(q6))/cos(q5))
    # where sin2 + cos2 = 1
    theta5 = atan2(sqrt(E3_6[1,0]**2 + E3_6[1,1]**2), E3_6[1,2]).evalf()
    theta5b = atan2(-sqrt(E3_6[1,0]**2 + E3_6[1,1]**2), E3_6[1,2]).evalf()

    # if(sin(theta5) < 0):
    #     theta4 = atan2(-E3_6[2,2], E3_6[0,2]).evalf()
    #     theta6 = atan2(E3_6[1,1], -E3_6[1,0]).evalf()
    # else:
    #     theta4 = atan2(E3_6[2,2], -E3_6[0,2]).evalf()
    #     theta6 = atan2(-E3_6[1,1], E3_6[1,0]).evalf()

    return [theta4, theta5, theta6]

def handle_calculate_IK(req):
    global numberOfRuns
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:
        # Initialize service response
        joint_trajectory_list = []
        for x in xrange(0, len(req.poses)):
            # IK code starts here
            joint_trajectory_point = JointTrajectoryPoint()

            # Define DH param symbols
            # a0, a1, a2, a3, a4, a5 = symbols('a0:6')
            # d1, d2, d3, d4, d5, d6, dG = symbols('d1:8')
            # alpha0, alpha1, alpha2, alpha3, alpha4, alpha5 = symbols('alpha0:6')
            # q1, q2, q3, q4, q5, q6 = symbols('q1:7') 

            # # Create individual transformation matrices
            # T0_1 =  Matrix([
            # [cos(q1), -sin(q1), 0,  0],
            # [sin(q1),  cos(q1), 0,  0],
            # [      0,        0, 1, d1],
            # [      0,        0, 0,  1]])

            # T1_2 =  Matrix([
            # [sin(q2),  cos(q2), 0, a1],
            # [      0,        0, 1,  0],
            # [cos(q2), -sin(q2), 0,  0],
            # [      0,        0, 0,  1]])

            # T2_3 =  Matrix([
            # [cos(q3), -sin(q3), 0, a2],
            # [sin(q3),  cos(q3), 0,  0],
            # [      0,        0, 1,  0],
            # [      0,        0, 0,  1]])

            # T3_4 =  Matrix([
            # [ cos(q4), -sin(q4), 0, a3],
            # [       0,        0, 1, d4],
            # [-sin(q4), -cos(q4), 0,  0],
            # [       0,        0, 0,  1]])

            # T4_5 =  Matrix([
            # [cos(q5), -sin(q5),  0, 0],
            # [      0,        0, -1, 0],
            # [sin(q5),  cos(q5),  0, 0],
            # [      0,        0,  0, 1]])

            # T5_6 =  Matrix([
            # [ cos(q6), -sin(q6), 0, 0],
            # [       0,        0, 1, 0],
            # [-sin(q6), -cos(q6), 0, 0],
            # [       0,        0, 0, 1]])

            # T6_G =  Matrix([
            # [0,  0, 1,  0],
            # [0, -1, 0,  0],
            # [1,  0, 0, dG],
            # [0,  0, 0,  1]])

            # T0_2 = T0_1*T1_2 #base_link to link_2
            # T0_3 = T0_2*T2_3 #link_2 to link_3
            # T0_4 = T0_3*T3_4 #link_3 to link_4
            # T0_5 = T0_4*T4_5 #link_4 to link_5
            # T0_6 = T0_5*T5_6 #link_5 to link_6
            
            # Extract end-effector position and orientation from request
            # px,py,pz = end-effector position
            # roll, pitch, yaw = end-effector orientation
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z

            P = np.array([[px], [py], [pz]])

            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w])
     
            T = tf.transformations.quaternion_matrix(
                [req.poses[x].orientation.x, 
                req.poses[x].orientation.y,
                req.poses[x].orientation.z, 
                req.poses[x].orientation.w]
                )
        
            R = T[:3,:3]

            dG = 0.303 #dG = d6+l
            # calulating the wrist center, only the first column is taken as the joints are along the axis
            # Looking for only lx, ly, lz instead of nx, ny, nz
            Wc = P - dG * R[:3, [0]]
            wx = Wc[0,0]
            wy = Wc[1,0]
            wz = Wc[2,0]

            [theta1, theta2, theta3, theta2b, theta3b] = getRRRJointAngles(wx,wy,wz)

            if theta2 == None:
                theta2 = theta2b
                theta3 = theta3b  

            elif theta2b != None:
                print("2 thetas")
                a0 = 0
                a1 = 0.35
                a2 = 1.25
                a3 = -0.054
                a4 = 0
                a5 = 0
                d1 = 0.75
                d2 = 0
                d3 = 0
                d4 = 1.5
                d5 = 0
                d6 = 0
                q1 = theta1
                q2 = theta2
                q3 = theta3
                q4 = 0
                q5 = 0
                q6 = 0
                dG = 0.303

                T0_5 = Matrix([
                [(sin(q1)*sin(q4) + sin(q2 + q3)*cos(q1)*cos(q4))*cos(q5) + sin(q5)*cos(q1)*cos(q2 + q3), -(sin(q1)*sin(q4) + sin(q2 + q3)*cos(q1)*cos(q4))*sin(q5) + cos(q1)*cos(q5)*cos(q2 + q3), -sin(q1)*cos(q4) + sin(q4)*sin(q2 + q3)*cos(q1), (a1 + a2*sin(q2) + a3*sin(q2 + q3) + d4*cos(q2 + q3))*cos(q1)],
                [(sin(q1)*sin(q2 + q3)*cos(q4) - sin(q4)*cos(q1))*cos(q5) + sin(q1)*sin(q5)*cos(q2 + q3), -(sin(q1)*sin(q2 + q3)*cos(q4) - sin(q4)*cos(q1))*sin(q5) + sin(q1)*cos(q5)*cos(q2 + q3),  sin(q1)*sin(q4)*sin(q2 + q3) + cos(q1)*cos(q4), (a1 + a2*sin(q2) + a3*sin(q2 + q3) + d4*cos(q2 + q3))*sin(q1)],
                [                                   -sin(q5)*sin(q2 + q3) + cos(q4)*cos(q5)*cos(q2 + q3),                                     -sin(q5)*cos(q4)*cos(q2 + q3) - sin(q2 + q3)*cos(q5),                            sin(q4)*cos(q2 + q3),           a2*cos(q2) + a3*cos(q2 + q3) + d1 - d4*sin(q2 + q3)],
                [                                                                                      0,                                                                                        0,                                               0,                                                             1]])

                opt1 = T0_5

                q1 = theta1
                q2 = theta2b
                q3 = theta3b

                T0_5 = Matrix([
                [(sin(q1)*sin(q4) + sin(q2 + q3)*cos(q1)*cos(q4))*cos(q5) + sin(q5)*cos(q1)*cos(q2 + q3), -(sin(q1)*sin(q4) + sin(q2 + q3)*cos(q1)*cos(q4))*sin(q5) + cos(q1)*cos(q5)*cos(q2 + q3), -sin(q1)*cos(q4) + sin(q4)*sin(q2 + q3)*cos(q1), (a1 + a2*sin(q2) + a3*sin(q2 + q3) + d4*cos(q2 + q3))*cos(q1)],
                [(sin(q1)*sin(q2 + q3)*cos(q4) - sin(q4)*cos(q1))*cos(q5) + sin(q1)*sin(q5)*cos(q2 + q3), -(sin(q1)*sin(q2 + q3)*cos(q4) - sin(q4)*cos(q1))*sin(q5) + sin(q1)*cos(q5)*cos(q2 + q3),  sin(q1)*sin(q4)*sin(q2 + q3) + cos(q1)*cos(q4), (a1 + a2*sin(q2) + a3*sin(q2 + q3) + d4*cos(q2 + q3))*sin(q1)],
                [                                   -sin(q5)*sin(q2 + q3) + cos(q4)*cos(q5)*cos(q2 + q3),                                     -sin(q5)*cos(q4)*cos(q2 + q3) - sin(q2 + q3)*cos(q5),                            sin(q4)*cos(q2 + q3),           a2*cos(q2) + a3*cos(q2 + q3) + d1 - d4*sin(q2 + q3)],
                [                                                                                      0,                                                                                        0,                                               0,                                                             1]])

                opt2 = T0_5
                
                difference1 = abs(wx - opt1[0,3]) + abs(wy - opt1[1,3]) + abs(wz - opt1[2,3])

                difference2 = abs(wx - opt2[0,3]) + abs(wy - opt2[1,3]) + abs(wz - opt2[2,3])
                
                if(difference1 > difference2):
                    theta2 = theta2b
                    theta3 = theta3b

            # Populate response for the IK request
            [theta4, theta5, theta6] = getRRRSpehericalArmJointAngles(yaw, pitch, roll, theta1, theta2, theta3)

            # theta6 = theta6 - np.pi/2
            # print("theta:", theta1, theta2,theta3,theta4,theta5,theta6)
            joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
            joint_trajectory_list.append(joint_trajectory_point)


        numberOfRuns += 1
        print("You have run ", numberOfRuns/2, " times with the number of commands:", len(joint_trajectory_list), ". Time to bring it home")
        rospy.loginfo("This is your new command: %s" % len(joint_trajectory_list))
        return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print "Ready to receive an IK request"
    rospy.spin()

if __name__ == "__main__":
    IK_server()