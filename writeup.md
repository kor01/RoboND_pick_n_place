## Project: Kinematics Pick & Place

[kr_fk]: ./misc_images/dh_frames_assignment.png

### Forward Kinematics Overview:

I solved forward kinematics by create a DH parameter table, whose parameters are converted from readings in URDF file

  * read relative joint position orientation and direction from URDF

  * construct absolute position orientation of each link's kinematics frame (absolute means represented in base_link frame)

  * compute the absolute representation of each joint axis

  * construct DH parameter table by DH frames constructed above


###  Read Joint Information from URDF:

  * read position and orientation of each joint from URDF file:

    read each xyz-rpy parameter from ```<joint> <origin>``` tag in URDF. Read axis direction from ```<joint> <axis> ``` tag.

    Joint | x | y | z | r | p | y | dx | dy | dz
    --- | --- | --- | --- | --- | --- | --- | --- | --- | ---
    1 | 0 | 0 | 0.33 | 0 | 0 | 0 | 0 | 0 | 1
    2 | 0.35 | 0 | 0.42 | 0 | 0 | 0 |  0 | 1 | 0
    3 | 0 | 0 | 1.25 | 0 | 0 | 0 | 0 | 1 | 0
    4 | 0.96 | 0 | -0.054 | 0 | 0 | 0 | 1 | 0 | 0
    5 | 0.54 | 0 | 0 | 0 | 0 | 0 | 0 | 1 | 0
    6 | 0.193 | 0 | 0 | 0 | 0 | 0 | 1 | 0 | 0

  * there is an implicit **kinematics frame** attached to each link:

      * in zero-pose, each frame can be obtained by a homogeneous transform from its previous frame.
      * the numerical value of the homogeneous transform can be obtained by xyz-rpy parameter in above table.
      * joint axis coordinate is described in its parent link's kinematics frame.
      * the code to obtain homogeneous transform from xyz-rpy parameter is in ```autofk/chain.py:9 rpy_to_homogenous()```


  * read information in URDF and create a corresponding python data structure is implemented in: ```autofk/parser.py: 134 URDFParser ```

  * the robot link structure is represented as a collection of interconnecting links and joints:

    ``` python
    from autofk.parser import URDFParser

    parser = URDFParser(path='kr210.urdf')
    for joint in parser.joints:
      print joint

    ```
  * the reason to create a parser instead of manually reading and hard-coding parameters is the parser allows us to automate forward kinematics construction (described blow)


### Compute Absolute Link Kinematics Frames and Joint Axis:

  * each link's kinematics frame in the above table is defined w.r.t the link's parent kinematics frame

  * it is hard to perform geometric analysis in this representation

  * we need to convert this representation to an **absolute representation** where each link's kinematics frame is described in the **world frame**.

  * In our case the **world frame** is chosen as the ```fixed_base_joint``` frame.

  * absolute frame representation construction:

      * convert each row's xyz-rpy in the above table to homogeneous transform matrix by applying ```autofk/chain.py:9 rpy_to_homogenous()```

      * start from identity homogeneous transform ```np.eye(4)```, successively multiply each homogeneous transform matrix obtained above.

  * KR210's zero-pose absolute kinematics frames (homogeneous transform matrix converted to xyz-rpy representation for compact display):

              Link | x | y | z | r | p | y | dx | dy | dz
              --- | --- | --- | --- | --- | --- | --- | --- | --- | ---
              0 | 0 | 0 | 0 | 0 | 0 | 0 | N/A | N/A | N/A
              1 | 0 | 0 | 0.33 | 0 | 0 | 0 | 0 | 0 | 1
              2 | 0.35 | 0 | 0.75 | 0 | 0 | 0 | 0 | 1 | 0
              3 | 0.35 | 0 | 2 | 0 | 0 | 0 | 0 | 1 | 0
              4 | 1.31 | 0 | 1.946 | 0 | 0 | 0 | 1 | 0 | 0
              5 | 1.85 | 0 | 1.946 | 0 | 0 | 0 | 0 | 1 | 0
              6 | 2.153 | 0 | 1.946 | 0 | 0 | 0 | 1 | 0 | 0

  * we notice that the above table can be obtained by simply sequentially add up each row's xyz in the original table.

  * this is because the rpy in the original table are all zero, these translation vectors are described the frames of the same orientation.

  * the code to perform the above transformation: ```autofk/chain.py: 23 infer_absolute_frames() ```

### DH Frames

 * DH Frames are the frames selected for each link

 * from DH-Frames we can derive DH-parameter table, which is a compact way to represent serial link structure and perform forward kinematics

* there are two set of rules. The basic rules define the basic requirement for a set of frames to be DH-frame. The advanced rules will favor the frame selection minimizing the number of non-zeros in DH-table.


### Basic Rules for Assigning DH Frames

the basic rules defines a legitimate DH frames assignment

  * for each link that is not base link and last link:

      * ```this-z``` should be its parent joint's joint axis

      * ```this-x``` should be a common normal between ```this-z``` and ```next-z```

      *  ```this-origin``` is the intersection between ```this-x``` and ```this-z```

  * last link:

      * ```this-z``` should be its parent joint's joint axis, same as above

      * ```this-x``` be arbitrary direction perpendicular to and intersecting with ```this-z``` (since there is no ```next-z``` for last link)

      * origin should be the intersection between ```this-z``` and ```this-x```

  * base link:

    * ```this-z``` can be arbitrary (since it has no parent joint)

    * ```this-x``` should be a common normal between ```this-z``` and ```next-z```


### Principle of Minimizing None-zeros:

in practice, the DH-frames assignment minimizing the non-zeros in DH-parameter table is favored

  * for each link that is not base link and last link:

      * if ```this-z``` and ```next-z``` are the same line, ```last-x``` is chosen as ```this-x```

      * if ```this-z``` and ```next-z``` are intersecting, ```this-x``` is determined up to a sign, choose the direction maximizing the cosine with ```last-x```

      * if ```this-z``` and ```next-z``` are not parallel, ```this-x``` has only one choice (the common normal between ```this-z``` and ```next-z```)

      * if ```this-z``` and ```next-z``` are parallel (but not the same line), the direction of ```this-x``` is **fixed** as the common normal between ```this-z``` and ```next-z```, **but the position is ambiguous**. In this case the origin is chosen as the intersection between ```last-x``` and ```this-z```.

  * last link:

      * ```last-x``` should be chosen as ```this-x```

  * base link:

      * ```next-z``` should be chosen as ```this-z```

      * the choice of base link x-axis's direction is complicated:

          * iterate through all its descendant joints, find the first joint axis that is not the same as ```this-z```. We denote the found axis as ```z-different```.

          * The direction of it's ```this-x``` should be the common normal between ```z-different``` and ```this-z```

          * iterate through all its descendant joints, find the first joint axis that is not parallel to ```this-z```. we denote the found axis as ```z-non-parallel```.

          * compute the common normal between ```z-non-parallel``` and ```this-z```, denoted as ```base-cn```

          * ```this-origin``` should be the **intersection**  between ```base-cn``` and ```this-z```


### DH-Frame Assignment for KR210

following the rules above, DH-Frame is chosen as the following schematic:
      ![alt text][kr_fk]

  * in this specific case(KR210): to compute the exact numerical value of each DH-frame matrix it is suffice to compute the coordinate of each origin

  * each origin is determined by the common normal between this-z and next-z

  * the absolute representation of each joint axis is already derived above, the numerical value of each x and o can be computed by applying function(in this KR210 case it can also be simply inferred by simple geometry):

      ``` python
      autofk/geometry.py: 122 common_normal()
      autofk/geometry.py: 132 common_normal_intersection()
      ```

* the result of DH-frame assignment for zero-pose robot

      Frame | o[0] | o[1] | o[2] | x[0] | x[1] | x[2] | z[0] | z[1] | z[2]
    --- | --- | --- | --- | --- | --- | --- | --- | --- | ---
    0 | 0 | 0 | 0.75 | 1 | 0 | 0 | 0 | 0 | 1
    1 | 0 | 1 | 0.75 | 1 | 0 | 0 | 0 | 0 | 1
    2 | 0.35 | 0 | 0.75 | 0 | 0 | 1 | 0 | 1 | 0
    3 | 0.35 | 0 | 2 | 0 | 0 | -1 | 0 | 1 | 0
    4 | 1.85 | 0 | 1.946 | 0 | 0 | -1 | 1 | 0 | 0
    5 | 1.85 | 0 | 1.946 | 0 | 0 | -1 | 0 | 1 | 0
    6 | 1.85 | 0 | 1.946 | 0 | 0 | -1 | 1 | 0 | 0






### DH-Parameter Table Rules:

from the above analysis we can compute the absolute homogeneous matrix of each DH-frame in the robot's zero pose (all joint coordinates = 0). In order to define a function maps joint coordinates to euclidean coordinates it is necessary to infer DH-parameter table from zero-pose frames:

  * start from ```1 to N```, each ```row[i]``` of DH-table represents a transform from ```link[i-1] to link[i]``` (links are zero-base indexed, from ```0 to N```)

  * ```row[i] = {alpha, a, d, theta}```

  * alpha is the twist angle from ```z[i-1] to z[i]``` w.r.t ```x[i-1]```

  * a is the distance from ```z[i-1] to z[i]``` w.r.t ```x[i-1]```

  * d is the distance from ```x[i-1] to x[i]``` w.r.t ```z[i]```

  * theta is the twist angle from ```x[i-1] to x[i]``` w.r.t ```z[i]```

  * the first two parameters determine ```z[i]``` from previous frame

  * the last two parameters determine ```x[i]``` from previous frame

  * either d or theta is symbolic value (determined by joint type), all other quantities are deterministic (determined by the robot's zero pose)

  * the initial value of the symbolic part can be determined by zero pose matrices we derived above


### DH-Parameter Table of KR210:

  by the above rule, we first infer the zero-pose value for joints:

      Joint | a | alpha | d | theta
        --- | --- | --- | --- | ---
        1 | 0 | 0 | 0 | 0
        2 | 0.35 | -pi/2 | 0 | -pi/2
        3 | 1.25 | 0 | 0 | 0
        4 | 0.056 | pi/2 | 1.5 | 0
        5 | 0 | -pi/2 | 0 | 0
        6 | 0 | pi/2 | 0 | 0

  this can be obtained by geometric inference from the schematics

  this can also be obtained by applying function: ```manipulator_graph/line_geometry.py: 13 angle_distance()``` on each consecutive x and z axis obtained in previous table

  since all of the joints are revolute, theta are the variable part of the table

### Symbolic Homogeneous Transforms

  the symbolic Homogeneous transform from last-frame to this-frame can be obtained by ```intrinsically applying```:

  * ```translation-x:a```  (translation about x axis, distance=a)

  * ``` rotation-x: alpha ```

  * ``` translation-z: d ```

  * ```rotation-z: theta ```

  once the DH-parameter table is obtained and the joint type is known, this transform can be **compiled into a function of ```theta or d```**

  ```manipulator_graph/forward_kinematics.py: 6 homogeneous_transform()``` the function use DH-parameter row to construct a symbolic transform matrix

  ``` manipulator_graph/forward_kinematics.py: 17 compile_homogenous() ``` compile the symbolic expression as a Fortran-Python module using sympy's autowrap feature.



### Individual Homogeneous Transforms of KR210:

  TBD

### Forward Kinematics Prediction:

the purpose of forward kinematics prediction is to map joint coordinates to euclidean coordinates

  * inputs: all joint states, the base_footprint frame (described in world frame). In the case of non-mobile robots the base_footprint frame is the world frame.

  * outputs: the value (described by a homogeneous transform matrix) of each DH-frame described in world frame.

  * the end-effector position and orientation can be extracted from the last DH-frame value (up to a fixed homogeneous transform)

the software implementation of this forward prediction is in:

  * ```manipulator_graph/serial_subgraph.py: 159 SerialSubGraph::forward() ```


### Forward Kinematics Implementation, Manually Implementation v.s. Automatic Approach:

To wrap up, as we discussed above, the forward kinematics of a robot can be constructed throw steps:

  1. Read ```<joint> <origin>``` and ```<joint> <axis> ``` from URDF file. We call the frame described in ```<joint> <origin>``` **kinematics frame**. Read the joint types from tag ```<joint> <type> ``` tag.

  2. read joint connection topology (in order to know parent-child relationship between joints)

  3. construct homogeneous transformations from the readings in ```step-1```

  4. construct ```zero-pose absolute frame value``` (frame described in world frame by a homogeneous transform matrix) for each kinematics frame

  5. construct **zero pose DH-frames** by the **basic rules** and **zero-minimizing principle** described above

  6. construct initial DH-parameter table from **zero pose DH-frames**

  7. compile each symbolic transform functions from  DH-parameter table and joint types

  8. implement a forward kinematics inference function, takes joint states and foot-frame of the robot and sequentially multiplies each transform to obtain numerical value of each DH-frame (and kinematics frame). The eef frame can be obtained by multiplying a fixed homogeneous transform to the last DH-frame

  these steps can be manually implemented case by case, by reading and filling value, performing geometric analysis over schematics, constructing and selecting common normals by experience.

  Manually implementation is educational, but in practice automatically constructing forward kinematics has following advantages:

  * it frees us from performing kinematics analysis repeatedly by following the same principles and rules every time.

  * it has better correctness and optimality guarantee.

  * it enables us to implement automatic inverse kinematics.

  In this project **I implemented the automatic forward kinematics inference**:

  1. parsing and extracting information from URDF file:

      - implemented in ```manipulator_graph/urdf_parser.py: 121 URDFParser```

      - parse from URDF file by:
          ``` python
          from manipulator_graph import URDFParser
          parser = URDFParser(path='my_good_robot.urdf')
          ```

      - parse from ros parameter server:
        ```python
        import rospy
        from manipulator_graph import URDFParser
        urdf = rospy.get_param('robot_description')
        parser = URDFParser(data=urdf)
          ```

      - accessing joint tag readings:

        ``` python
        for joint in parser.joints:
          print joint
        ```

  2.  construct forward kinematics:

      ```python
      from manipulator_graph.kinematics_dag import KinematicsDag
      from manipulator_graph import URDFParser
      parser = URDFParser(path='my_good_robot.urdf')
      dag = KinematicsDag(parser.joints)
      graph = dag.root.graph
      ```
  3. forward kinematics inference:

      ```python
      from manipulator_graph.kinematics_dag import KinematicsDag
      from manipulator_graph import URDFParser
      import tf # to broadcast DH-frames

      br = tf.TransformBroadcaster()

      base_name, _ = graph.dh_base_link
      joint_states = ... # whatever joint states from ros

      # return a tuple of (link_name, frame_matrix) of each link
      frames = graph.forward(joint_states, return_dh=True)

      # iterate through frames an publish to tf
      for link_name, frame_value in frames:
        p, q = rot_to_quaternion(frame_value)
        name = 'dh-' + link_name
        br.sendTransform(
          p, q, rospy.Time.now(), name, base_name)

      ```

  what happened under the hood:

  * when calling the constructor ```KinematicsDag(parser.joints)```:

      1. the topology of joints is analyzed, and the joint set is partitioned as a DAG:

          - each node in the DAG represents a linear chain of joints, i.e. a consecutive serial connected joints
          - node ```a and b``` are connected iff a's last link is the parent of b's first link
          - nodes in DAG are rigidly connected (i.e. by a fixed joint)
          - the partition into DAG is implemented in ```manipulator_graph/kinematics_dag.py: partition_as_serial_dag()```
      2. each serial part


### Inverse Kinematics Terminology:

  * waist is referred to as the first joint, since it controls the x-y-planar rotation of the robot list human waist

  * shoulder and elbow are referred to as the second and third joints for the same anthropic reason above

  * wrist is referred to as the collection of the last three joints, since it controls the orientation of the end effector like human wrist

  * wrist center is referred to as the intersection of the three wrist axis


### Inverse Kinematics Analysis:

  * over all strategy

  * infer the last DH-frame position and orientation from eef

  * infer the wrist joint states from the last DH-frame

  * infer the wrist center location from the last DH-frame

  * infer the waist joint state from the wrist center

  * infer the shoulder and elbow angle by combining the waist state and wrist center


### Forward Kinematics Implementation

1. Generic automatic DH parameter table inference from URDF file

2. Generic forward kinematics inference

2. Geometric inverse kinematics for kr210 kuka arm


### Inverse Kinematics Implementation

---

### Experiments

  * forward kinematics debug and validation


  * inverse kinematics server pick and place


### Run the project:


1. install project dependency: pip install gflags
2. Set up your ROS Workspace.
3. Download or clone the [project repository](https://github.com/udacity/RoboND-Kinematics-Project) into the ***src*** directory of your ROS Workspace.  
4. copy kuka_kinematics, manipulator_graph, and ks_runner.sh to kuka_arm/script directory
5. roscd kuka_arm/script; sh target_spawn.py to start environment
6. sh ks_runner.sh to start kinematics server


[//]: # (Image References)

[image1]: ./misc_images/dh_simple.png
[image2]: ./misc_images/dh_all.png
[rviz_grip]: ./misc_images/rviz_grip.png
[gazebo_grip]: ./misc_images/gazebo_grip.png
[ks_log]: ./misc_images/ks_log.png

---

### [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points


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
