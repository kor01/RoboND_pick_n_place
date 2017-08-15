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

[image1]: ./misc_images/dh_simple.png
[image2]: ./misc_images/dh_all.png
[rviz_grip]: ./misc_images/rviz_grip.png
[gazebo_grip]: ./misc_images/gazebo_grip.png
[ks_log]: ./misc_images/ks_log.png

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


7.  the DH parameter obtained from automatic inference:

    Joint | a | alpha | d | theta
    --- | --- | --- | --- | ---
    1 | 0 | 0 | 0 | 0
    2 | 0.35 | 1.5707963 | 0.0 | -1.570796
    3 | 1.25 | 0 | 0 | 3.141592
    4 | 0.54 | 1.5707963 | 1.5 | 3.1415926
    5 | 0 | 1.5707963 | 0 | 3.1415926
    6 | 0 | 1.5707963 | 0 | 0

    as illustrated in notebook/kinematics_dag.ipnb


#### Inverse Kinematics:

 1. inverse kinematics implementation is not generic and is specific to KR210 arm

 2. the geometric method:

     a. convert the EEF frame in query into target DHFrame of gripper link

     b. use the EEF frame Z axis and wrist center offset to infer the wrist center location

     c. use wrist center location to solve the waist joint angle

     d. use the waist location combined with wrist location to solve elbow and
     shoulder angle by trigonometric equations

     e. choose valid solution by joint limit


#### Experiment:

  1. the correctness of forward kinematics is validated by broadcast DH frames
  derived from joint states by TF, and validate in RViz, as illustrated by:

      validate DH frames under different joint angle configuration

      execute command: ***python -m kuka_kinematics***

      to broadcast DH frames using TF


      dh parameter validation using TF and Rviz
  ![alt text][image1]


      all dh frames broadcast into Rviz
  ![alt text][image2]


  2. the correctness of inverse kinematics in validated by gripping task execution;
  the planned trajectory is correctly executed without any observable error

      run kinematics server by steps in ***run the project*** section


    rviz view

  ![grip in rviz][rviz_grip]


    gazebo view

  ![grip in gazebo][gazebo_grip]

    kinematics server log, trajectory error, solution selection and execution time

  ![kinematics server log][ks_log]
