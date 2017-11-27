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
from IK_class import *
from FK_class import *
import matplotlib.pyplot as plt

def handle_calculate_IK(req):
    global error
    plt.close('all')
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:

        ### Your FK code here
        # Create symbols
        q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')

        # Correction needed to account of correlation between URDF and DH convention
#        dh_kuka = DH()
#        fk_kuka = FK(dh_kuka)
#        ik_kuka = IK()
    # Create Modified DH parameters

    # Define Modified DH Transformation matrix
        # Homogeneous Transforms

    # Create individual transformation matrices
        # Composition of Homogeneous Transforms

    # Correction needed to account of correlation between URDF and DH convention

    # Extract rotation matrices from the transformation matrices


        ###
        # Initialize service response
        joint_trajectory_list = []
        global eef_pose_error
#        print req.poses
        for x in xrange(0, len(req.poses)):
            # IK code starts here
            joint_trajectory_point = JointTrajectoryPoint()

        # Extract end-effector position and orientation from request
        # px,py,pz = end-effector position
        # roll, pitch, yaw = end-effector orientation
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z

#            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
#                [req.poses[x].orientation.x, req.poses[x].orientation.y,
#                    req.poses[x].orientation.z, req.poses[x].orientation.w])
            (roll, pitch, yaw) = tf.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w])

            ### Your IK code here
        # Compensate for rotation discrepancy between DH parameters and Gazebo
            ik_kuka.get_Rrpy(roll, pitch, yaw)
            wc = ik_kuka.get_wc(px,py,pz)
#            print wc
            tt1 = ik_kuka.get_Theta1()
            tt2_3 = ik_kuka.get_Theta2_3(wk=0)
            T3_0 = fk_kuka.invT(fk_kuka.T0_3)
            tt4_6 = ik_kuka.get_orientation((tt1[0],tt2_3[0],tt2_3[1]),T3_0)


            ## Insert IK code here!
            theta1 = min(tt1)
            if theta1 < -1:
                theta1 = tt1[0]
            if fk_kuka.verify_rich_wc(theta1,tt2_3[0],tt2_3[1],wc):
                theta2 = tt2_3[0]
                theta3 = tt2_3[1]
            else:
                tt2_3 = ik_kuka.get_Theta2_3(wk=0,cfg=0)
                print "try other cfg"
                if fk_kuka.verify_rich_wc(theta1,tt2_3[0],tt2_3[1],wc):
                     theta2 = tt2_3[0]
                     theta3 = tt2_3[1]
                else:
                    print "used last poses"
                    theta2 = joint_trajectory_list[-1].positions[1]
                    theta3 = joint_trajectory_list[-1].positions[2]


            T3_0 = fk_kuka.invT(fk_kuka.T0_3)
            tt4_6 = ik_kuka.get_orientation((theta1,tt2_3[0],tt2_3[1]),T3_0)
            theta4 = tt4_6[0]
            theta5 = tt4_6[1]
            theta6 = tt4_6[2]



        # Calculate joint angles using Geometric IK method
        #
        #
            ###

            # Populate response for the IK request
            # In the next line replace theta1,theta2...,theta6 by your joint angle variables
            joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
            joint_trajectory_list.append(joint_trajectory_point)
            T = fk_kuka.T0_EE.evalf(subs={q1:theta1,q2:theta2,q3:theta3,q4:theta4,q5:theta5,q6:theta6})
            error = sqrt((px-T[0,3])**2+ (py-T[1,3])**2+(pz-T[2,3])**2)
            eef_pose_error.append(error)


        rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
        rospy.loginfo("average end-effector pose error FK/IK: %f" % (1.0* sum(eef_pose_error)/len(eef_pose_error)))
        return CalculateIKResponse(joint_trajectory_list)



def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print "Ready to receive an IK request"
#    pub_eef_error(error)
    rospy.spin()

if __name__ == "__main__":
    eef_pose_error = []
    dh_kuka = DH()
    fk_kuka = FK(dh_kuka)
    ik_kuka = IK()
    IK_server()
    plt.figure(figsize=(8,6))
    plt.plot(eef_pose_error)
    plt.xlabel('Poses calculated', fontsize=12)
    plt.ylabel('Error',fontsize=12)
    plt.title('End-effector pose error',fontsize=16)
    plt.show()
#    pub_eef_error(error)
#