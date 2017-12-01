## Project: Kinematics Pick & Place

[//]: # (Image References)

[image1]: ./misc_images/misc1.png
[image2]: ./misc_images/misc3.png
[image3]: ./misc_images/misc2.png
[image4]: ./misc_images/kuka-arm-diagram.jpg

## Rubric Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it!

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

Forward kinematics is concerned with calculating the pose of an object given the states of its kinematic parameters. In this project, modified Denavit-Hartenberg(DH) parameters are used to associate reference frames to each link. Names and definitions for the parameters are included in Appendix A. For the KR210 serial manipulator used in this project, those parameters include the twist angles, link length, link offset, and joint angles. These parameters are then used to create transform equations to describe the translation and orientation of one link in the manipulator relative to another. The diagram below shows the parameters related to the KR210 robot arm.

![alt text][image4]

These parameters were determined following the process outlined in appendix B. An offset of -90 degrees is necessary for theta 2 due to the physical layout of the KR210 arm.


Links | alpha(i-1) | a(i-1) | d(i-1) | theta(i)
---   | ---        | ---    | ---    | ---
0->1  | 0          | 0      | 0.75   | q1
1->2  | -pi/2      | 0.35   | 0      | -pi/2 + q2
2->3  | 0          | 1.25   | 0      | q3
3->4  | -pi/2      | -0.054 | 1.50   | q4
4->5  | pi/2       | 0      | 0      | q5
5->6  | -pi/2      | 0      | 0.303  | q6
6->Gripper | 0          | 0      | 0.303  | 0


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

Multiplying these individual matrices gives us the composition of homogeneous transforms from the base link to the gripper.

#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics

The inverse kinematics problem to determine the required joint angles to obtain a desired gripper pose is split into two parts; inverse position and inverse orientation. Given the configuration of the KR210 arm, the first 3 joint angles can be used to determine the x,y, and z values for the gripper wrist center. The final 3 joints form what is called a spherical wrist, which determines the orientation of the grippers roll, pitch, and yaw.

Theta 1 can be solved for by projecting the wrist center coordinates onto the ground x-y plane. We can then use the xy coordinates and the atan2 function to calculate the angle of theta 1.

	θ1 = atan2(Xc,Yc)


![alt text][image2]
Theta 2 and theta 3 can be better understood by referring to the above picture, showing both joints projected onto the x-z plane.

First, the triangle formed by joint 2, joint 3 and the wrist center is solved. Two of the sides correspond to DH parameters determined earlier

	sideA = 1.501 #DH parameter d4
	sideC = 1.25 #DH parameter a2
The third side can be found by creating a new right triangle with vertices at J2, wrist center, and the point at wrist center x and joint 2 z. The third side can then be solved as the hypotenuse of this new triangle.

	sideB = sqrt(pow((sqrt(WC[0]WC[0]+WC[1]*WC[1])-0.35),2)+pow((WC[2]-0.75),2))

The corresponding angles using the side lengths and the arc cos function.

	angA = acos((sideB*sideB+sideC*sideC-sideA*sideA)/(2*sideB*sideC))
	angB = acos((sideA*sideA+sideC*sideC-sideB*sideB)/(2*sideA*sideC))
	angC = acos((sideA*sideA+sideB*sideB-sideC*sideC)/(2*sideA*sideB))

Theta 2 can be solved by using angA and solving for the angle formed by WC, J2, and WCxJ2z and relative to 90 degrees vertical

	theta2 = pi/2 - angA - atan2(WC[2]-0.75,sqrt(WC[0]*WC[0]+WC[1]*WC[1])-0.35)

Theta 3 can then be solved using angB and the 90 degree vertical.

	theta3 = pi/2 - (angB + 0.036) #0.036 adjusts for -0.054m drop in link 4

With the first 3 joint angles solved to give our wrist center position, we can move on to solve the final 3 joints for orientation.
First we substitute the values for the first 3 joints into our complete transform. Then we use the inverse of the rotational component of the transform up to J3 to determine the difference in orientation that needs to be accounted for in J4-J6. By examining the rotational component of the symbolic transform from J4-J6 show below, we can solve for the remaining joint angles using the atan2 function.


	theta4 = atan2(R3_6[2,2], -R3_6[0,2])
	theta5 = atan2(sqrt((R3_6[1,0]*R3_6[1,0] + R3_6[1,1]*R3_6[1,1])),R3_6[1,2])
	theta6 = atan2(-R3_6[1,1], R3_6[1,0])


### Project Implementation

After using the IK_debug routine to check and refine the methods discussed previously to solve the kinematics problem, I implemented the appropriate parts into the IK server code. Based on trial and error and mentorship in the classroom, I changed the method to obtain the inverse of the rotational transform matrix from J1-J3 from LU to using the transpose. While the implementation is able to perform the kinematics calculations with an acceptable level of error, there is still several areas where the implementation can be improved. The solution is not necessarily robust in dealing with potential kinematic singularities or using the optimal solution under conditions of multiple solutions.Another area of improvement would be reducing extraneous rotations when moving from between points on the calculated path. Along with this would be improving the error present in the kinematic solution.


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
