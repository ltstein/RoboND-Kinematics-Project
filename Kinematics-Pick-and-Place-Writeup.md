## Project: Kinematics Pick & Place

[//]: # (Image References)

[image1]: ./misc_images/misc1.png
[image2]: ./misc_images/misc3.png
[image3]: ./misc_images/misc2.png

## Rubric Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it!

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

Forward kinematics is concerned with calculating the pose of an object given the states of its kinematic parameters. In this project, modified Denavit-Hartenberg(DH) parameters are used to associate reference frames to each link. Names and definitions for the parameters are included in Appendix A. For the KR210 serial manipulator used in this project, those parameters include the twist angles, link length, link offset, and joint angles. These parameters are then used to create transform equations to describe the translation and orientation of one link in the manipulator relative to another. The diagram below shows the parameters related to the KR210 robot arm.

--INSERT DH PARAMETER DIAGRAM--

These parameters were determined following the process outlined in appendix B.

Here is an example of how to include an image in your writeup.

![alt text][image1]

Links | alpha(i-1) | a(i-1) | d(i-1) | theta(i)
---   | ---        | ---    | ---    | ---
0->1  | 0          | 0      | 0.75   | q1
1->2  | -pi/2      | 0.35   | 0      | -pi/2 + q2
2->3  | 0          | 1.25   | 0      | q3
3->4  | -pi/2      | -0.054 | 1.50   | q4
4->5  | pi/2       | 0      | 0      | q5
5->6  | -pi/2      | 0      | 0.303  | q6
6->Gripper | 0          | 0      | 0.303  | 0

#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

Your writeup should contain individual transform matrices about each joint using the DH table and a homogeneous transform matrix from base_link to gripper_link using only the position and orientation of the gripper_link. These matrices can be created using any software of your choice or hand written. Also include an explanation on how you created these matrices.

The following are individual transformation matrices between each link on the KR210. These were generated using the IK_debug program but are also obtainable using the general transformation and substituting the values from the DH parameter table.

T0_1=
[cos(q1), -sin(q1), 0,  0.0],
[sin(q1),  cos(q1), 0,    0],
[      0,        0, 1, 0.75],
[      0,        0, 0,    1]

T1_2=
[ cos(q2 - 0.5*pi), -sin(q2 - 0.5*pi), 0, 0.35],
[                0,                 0, 1,  0.0],
[-sin(q2 - 0.5*pi), -cos(q2 - 0.5*pi), 0,    0],
[                0,                 0, 0,    1]

T2_3=
[cos(q3), -sin(q3), 0, 1.25],
[sin(q3),  cos(q3), 0,    0],
[      0,        0, 1,  0.0],
[      0,        0, 0,    1]

T3_4=
[ cos(q4), -sin(q4), 0, -0.054],
[       0,        0, 1,    1.5],
[-sin(q4), -cos(q4), 0,      0],
[       0,        0, 0,      1]

T4_5=
[cos(q5), -sin(q5),  0, 0.0],
[      0,        0, -1,   0],
[sin(q5),  cos(q5),  0,   0],
[      0,        0,  0,   1]

T5_6=
[ cos(q6), -sin(q6), 0,   0.0],
[       0,        0, 1, 0.303],
[-sin(q6), -cos(q6), 0,     0],
[       0,        0, 0,     1]

T6_G=
[1, 0, 0,   0.0],
[0, 1, 0,     0],
[0, 0, 1, 0.303],
[0, 0, 0,     1]

Multiplying these individual matrices gives us the composition of homogeneous transforms from the base link to the gripper, shown below.

T0_G=




#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

And here's where you can draw out and show your math for the derivation of your theta angles.

The inverse kinematics problem to determine the required joint angles to obtain a desired gripper pose is split into two parts; inverse position and inverse orientation. Given the configuration of the KR210 arm, the first 3 joint angles can be used to determine the x,y, and z values for the gripper wrist center. The final 3 joints form what is called a spherical wrist, which determines the orientation of the grippers roll, pitch, and yaw.

Theta 1 can be solved for by projecting the wrist center coordinates onto the ground x-y plane. We can then use the xy coordinates and the atan2 function to calculate the angle of theta 1.

	θ1 = atan2(Xc,Yc)


![alt text][image2]
Theta 2 and theta 3 can be better understood by referring to the above picture, showing both joints projected onto the x-z plane.

First, the triangle formed by joint 2, joint 3 and the wrist center is specified

sideA = 1.501 #DH parameter d4
	sideC = 1.25 #DH parameter a2
	sideB = sqrt(pow((sqrt(WC[0]WC[0]+WC[1]*WC[1])-0.35),2)+pow((WC[2]-0.75),2))

	angA = acos((sideB*sideB+sideC*sideC-sideA*sideA)/(2*sideB*sideC))
	angB = acos((sideA*sideA+sideC*sideC-sideB*sideB)/(2*sideA*sideC))
	angC = acos((sideA*sideA+sideB*sideB-sideC*sideC)/(2*sideA*sideB))


	#theta 2
	theta2 = pi/2 - angA - atan2(WC[2]-0.75,sqrt(WC[0]*WC[0]+WC[1]*WC[1])-0.35)
	#
	theta3 = pi/2 - (angB + 0.036) #0.036 adjusts for -0.054m drop in link 4
	#

Based on the geometric Inverse Kinematics method described here, breakdown the IK problem into Position and Orientation problems. Derive the equations for individual joint angles. Your writeup must contain details about the steps you took to arrive at those equations. Add figures where necessary. If any given joint has multiple solutions, select the best solution and provide explanation about your choice (Hint: Observe the active robot workspace in this project and the fact that some joints have physical limits).

### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results.


Here I'll talk about the code, what techniques I used, what worked and why, where the implementation might fail and how I might improve it if I were going to pursue this project further.  


And just for fun, another example image:
![alt text][image3]

Project Implementation
Criteria 	Meets Specifications

Fill in the IK_server.py file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles.

IK_server.py must contain properly commented code. The robot must track the planned trajectory and successfully complete pick and place operation. Your writeup must include explanation for the code and a discussion on the results.

To have a standout submission, calculate and plot the error in end-effector pose generated by your joint angle commands. You can do this by calculating end-effector poses via Forward Kinematics using your code output (which is a set of joint angles) and comparing it with the end-effector poses you received as input.

### Appendix A
## DH Paremeter Definitions
α ​i−1​​ (twist angle) = angle between ​Z​^​​​i−1​​ and ​Z​^​​​i​​ measured about ​X​^​​​i−1​​ in a right-hand sense.

a ​i−1​​ (link length) = distance from ​Z​^​​​i−1​​ to ​Z​^​​​i​​ measured along ​X​^​​​i−1​​ where ​X​^​​​i−1​​ is perpendicular to both ​Z​^​​​i−1​​ to ​Z​^​​​i​​

d​i​​ (link offset) = signed distance from ​X​^​​​i−1​​ to ​X​^​​​i​​ measured along ​Z​^​​​i​​. Note that this quantity will be a variable in the case of prismatic joints.

θ​i​​ (joint angle) = angle between ​X​^​​​i−1​​ to ​X​^​​​i​​ measured about ​Z​^​​​i​​ in a right-hand sense. Note that this quantity will be a variable in the case of a revolute joint.

### Appendix B
## Denavit-Hartenberg Parameter Assignment Algorithm

1)  Label all joints from {1, 2, … , n}.

2)  Label all links from {0, 1, …, n} starting with the fixed base link as 0.

3)  Draw lines through all joints, defining the joint axes.

4)  Assign the Z-axis of each frame to point along its joint axis.

5)  Identify the common normal between each frame ​Z​^​​​i−1​​ and frame ​Z​^​​​i​​ .

6)  The endpoints of "intermediate links" (i.e., not the base link or the end effector) are associated with two joint axes, {i} and {i+1}. For i from 1 to n-1, assign the ​X​^​​​i​​ to be:

a) For skew axes, along the normal between ​Z​^​​​i​​ and ​Z​^​​​i+1​​ and pointing from {i} to {i+1}.

b) For intersecting axes, normal to the plane containing ​Z​^​​​i​​ and ​Z​^​​​i+1​​.

c) For parallel or coincident axes, the assignment is arbitrary; look for ways to make other DH parameters equal to zero

7)  For the base link, always choose frame {0} to be coincident with frame {1} when the first joint variable (θ​1​​ or d​1​​) is equal to zero. This will guarantee that α​0​​ = a​0​​ = 0, and, if joint 1 is a revolute, d​1​​ = 0. If joint 1 is prismatic, then θ​1​​= 0.

8) For the end effector frame, if joint n is revolute, choose X​n​​ to be in the direction of X​n−1​​ when θ​n​​ = 0 and the origin of frame {n} such that d​n​​ = 0.
