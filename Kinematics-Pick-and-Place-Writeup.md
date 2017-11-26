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

Forward kinematics is concerned with calculating the pose of an object given the states of its kinematic parameters. For the KR210 serial manipulator used in this project, those parameters include the joint angles, orientation, and arm segment lengths.

In this project, modified Denavit-Hartenberg(DH) parameters are used to associate reference frames to each link. These parameters are then used to create transform equations to describe the translation and orientation of one link in the manipulator relative to another. The diagram below shows the parameters related to the KR210 robot arm.


These parameters were determined following the process outlined in appendix ##.


Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

Your writeup should contain a DH parameter table with proper notations and description about how you obtained the table. Make sure to use the modified DH parameters discussed in this lesson. Please add an annotated figure of the robot with proper link assignments and joint rotations (Example figure provided in the writeup template). It is strongly recommended that you use pen and paper to create this figure to get a better understanding of the robot kinematics.

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
6->EE | 0          | 0      | 0.303  | 0

#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

Your writeup should contain individual transform matrices about each joint using the DH table and a homogeneous transform matrix from base_link to gripper_link using only the position and orientation of the gripper_link. These matrices can be created using any software of your choice or hand written. Also include an explanation on how you created these matrices.

#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

And here's where you can draw out and show your math for the derivation of your theta angles.

![alt text][image2]

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
