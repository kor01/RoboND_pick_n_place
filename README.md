## Project: Kinematics Pick & Place
###  implementation Specs:

1. Generic automatic DH parameter table inference from URDF file

2. Generic forward kinematics inference

2. Geometric inverse kinematics for kr210 kuka arm
---

**Run the project:**  
0. install project dependency: pip install gflags
1. Set up your ROS Workspace.
2. Download or clone the [project repository](https://github.com/udacity/RoboND-Kinematics-Project) into the ***src*** directory of your ROS Workspace.  
3. copy kuka_kinematics, manipulator_graph, and ks_runner.sh to kuka_arm/script directory
4. roscd kuka_arm/script; sh target_spawn.py to start environment
5. sh ks_runner.sh to start kinematics server


[//]: # (Image References)

[image1]: ./misc_images/misc1.png
[image2]: ./misc_images/misc2.png
[image3]: ./misc_images/misc3.png

## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points

### Kinematic Analysis


#### DH Parameter and DH Frame Inference:

 1. I have implemented a generic package automatically infer the DH 
 frame assignment and DH parameter directly from URDF files
 
 2. jupyter notebook of DH parameters and DH frames inference from URDF file
 
#### Forward Kinematics:

 1. I also have implemented a generic forward kinematics package
 
 2. using the analysis from the first package and build automatic forward kinematics
 
 3. the package can infer both the DH frames of each link and the URDF frames of each link
 
 4. the package provides convenient API to aid inverse kinematics implementation 

 5. the package uses sympy autowrap to speed up inference computation
 
 6. the package automatically check joint limit (throw exception if unreachable)

#### Inverse Kinematics:

 1. inverse kinematics implementation is not generic and is specific to KR210 arm
 
 2. the geometric method: 
 
     a. convert the EEF frame in query into target DHFrame of gripper link
     
     b. use the EEF frame Z axis and wrist center to 

Here is an example of how to include an image in your writeup.

![alt text][image1]

#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

Here's | A | Snappy | Table
--- | --- | --- | ---
1 | `highlight` | **bold** | 7.41
2 | a | b | c
3 | *italic* | text | 403
4 | 2 | 3 | abcd

#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

And here's another image! 

![alt text][image2]

### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 


Here I'll talk about the code, what techniques I used, what worked and why, where the implementation might fail and how I might improve it if I were going to pursue this project further.  


And just for fun, another example image:
![alt text][image3]


