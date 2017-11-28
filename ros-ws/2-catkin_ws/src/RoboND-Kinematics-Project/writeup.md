## Project: Kinematics Pick & Place
### Writeup Template: You can use this file as a template for your writeup if you want to submit it as a markdown file, but feel free to use some other method and submit a pdf if you prefer.

---


**Steps to complete the project:**  


1. Set up your ROS Workspace.
2. Download or clone the [project repository](https://github.com/udacity/RoboND-Kinematics-Project) into the ***src*** directory of your ROS Workspace.  
3. Experiment with the forward_kinematics environment and get familiar with the robot.
4. Launch in [demo mode](https://classroom.udacity.com/nanodegrees/nd209/parts/7b2fd2d7-e181-401e-977a-6158c77bf816/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/91d017b1-4493-4522-ad52-04a74a01094c/concepts/ae64bb91-e8c4-44c9-adbe-798e8f688193).
5. Perform Kinematic Analysis for the robot following the [project rubric](https://review.udacity.com/#!/rubrics/972/view).
6. Fill in the `IK_server.py` with your Inverse Kinematics code.


[//]: # (Image References)

[image1]: ./misc_images/fk_img_1.png
[image2]: ./misc_images/fk_img_2.png
[image3]: ./misc_images/dh-parameter.jpg

## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  


### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

![alt text][image1]
![alt text][image2]

From the forward_kinematic demo, my first step was to analyze the URDF.xacro file. Shortly I realized that the same information at the URDF file it is available at RViz in the Display tree inside the TF parent, so I began to get the distances between joints from there to create the DH-parameter table. I also used the demo to analyze the signs of the rotation of each joint.

In the following figure, the DH parameters of the robot are schematized.
![alt text][image3]

The resulting Table:
Joint | Links | alpha(i-1)  | a(i-1) | d(i) | theta(i)
  --- | ---   | ---         | ---    | ---    | ---
1     |0->1  | 0             | 0      | 0.75   | $\theta_1$
2     |1->2  | $- \pi/2$   | 0.35   | 0      | -pi/2 + $\theta_2$
3     |2->3  | 0           | 1.25   | 1.5    | $\theta_3$
4     |3->4  | $- \pi/2$   | -0.54  | 0      | $\theta_4$
5     |4->5  | $\pi/2$     | 0      | 0      | $\theta_5$
6     |5->6  | $- \pi/2$   | 0      | 0      | $\theta_6$
7     |6->EE | 0           | 0      | 0.303  | 0

* alpha(i-1): twist angle, angle between axis Z(i-1) and Z(i) measured about axis X(i-1)
* a(i-1): link length, distance from axis Z(i-1) to Z(i) measured along axis X(i-1)
* d(i): link offset, distance from axis X(i-1) to X(i) measured along axis Z(i)
* theta(i): joint angle, angle between axis X(i-1) and X(i) measured about axis Z(i)

As it was explained in the lessons, the only thing that need attention is the offset of Theta 2 by 90 degrees.

#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

Following the Modified Denavit-Hartenberg parameters, a homogeneous transformation
it is defined as


#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

And here's where you can draw out and show your math for the derivation of your theta angles.


### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results.


Here I'll talk about the code, what techniques I used, what worked and why, where the implementation might fail and how I might improve it if I were going to pursue this project further.  


And just for fun, another example image:


## References
[1] John J. Craig, Introduction to Robotics: Mechanics and Control (3rd Edition) ISBN 978-0201543612
