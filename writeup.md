## Project: Kinematics Pick & Place

[kr_fk]: ./misc_images/dh_frames_assignment.png
[homogenous]: ./misc_images/individual_homogenous.png
[geo_ik]: ./misc_images/geometric_ik.png
[multi_solution]: ./misc_images/multiple_solutions.png
[fk_intial]: ./misc_images/fk_initial.png
[fk_move]: ./misc_images/fk_move.png
[fk_model]: ./misc_images/fk_model.png

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

 * from DH-Frames we can derive DH-parameter table, which is a compact parameterization of serial link structure.

* there are two set of rules. The **basic rules** define the basic requirement for a set of frames to be DH-frame. The **advanced rules** will favor the frame selection **minimizing the number of non-zeros** in DH-table.


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

  * the absolute representation of each joint axis is already derived above, the numerical value of each x and o can be computed by applying function:

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

* the implementation of the above rules can be found in: ```autofk/dh.py 121 assign_dh_frames() ```, will be elaborated later in this writeup.


### DH-Parameter Table Rules:

from the above analysis we can compute the absolute homogeneous matrix of each DH-frame in the robot's zero pose (all joint coordinates = 0).

In order to define a **function maps joint coordinates to euclidean coordinates** it is necessary to infer DH-parameter table from zero-pose frames:

  * start from ```1 to N```, each ```row[i]``` of DH-table represents a transform from ```link[i-1] to link[i]``` (links are zero-base indexed, from ```0 to N```)

  * ```row[i] = {alpha, a, d, theta}```

  * ```alpha``` is the twist angle from ```z[i-1] to z[i]``` w.r.t ```x[i-1]```

  * ```a``` is the distance from ```z[i-1] to z[i]``` w.r.t ```x[i-1]```

  * ```d``` is the distance from ```x[i-1] to x[i]``` w.r.t ```z[i]```

  * ```theta``` is the twist angle from ```x[i-1] to x[i]``` w.r.t ```z[i]```

  * the first two parameters determine ```z[i]``` from previous frame

  * the last two parameters determine ```x[i]``` from previous frame

  * either ```d``` or ```theta``` is symbolic value (determined by joint type), all other quantities are deterministic (determined by the robot's zero pose)

  * the initial value of the symbolic part can be determined by zero pose matrices we derived above


### DH-Parameter Table of KR210:

  since all joints in our KR210 are revolute joints, the joint parameter vector is ```[theta1, ... , theta6]```

  by the above rule, we first infer the zero-pose value for joints:

    Joint | a | alpha | d | theta
    --- | --- | --- | --- | ---
    1 | 0 | 0 | 0 | ```theta1```
    2 | 0.35 | -pi/2 | 0 | -pi/2 + ```theta2```
    3 | 1.25 | 0 | 0 | ```theta3```
    4 | 0.056 | pi/2 | 1.5 | ```theta4```
    5 | 0 | -pi/2 | 0 | ```theta5```
    6 | 0 | pi/2 | 0 | ```theta6```

  the implementation of constructing DH-parameter from joint type and initial DH-frame can be found in ```autofk/dh.py:170  infer_dh_parameters() ```, which takes frames obtained in ```assign_dh_frames()``` and return a ```list``` of ```autofk/dh.py:47 DHRow```.

  the algorithm to compute initial DH-parameter value is simply applying ```autofk/geometry.py:46 angle_distance()```  which computes the angle and distance between two lines w.r.t a direction.

  * Application on ```last-z``` and ```this-z``` w.r.t ```last-x ``` will derive ```a``` and ```alpha```.
  * Application on ``` last-x``` and ```this-x``` w.r.t ```this-z``` will derive ```d``` and ```theta```

  once the initial value of the table is obtained, a symbolic structure can be built. implemented in ```autofk/chain.py:60 construct_dh_links()```:
  * the function constructs a list of ```autofk/link.py:29 DHLink``` which computational workhouse for our forward kinematics inference.

  * inside each ```DHLink```, a symbolic homogeneous transform is created by its initial DH-table values and joint type.

  * the symbolic matrix is compiled as a Fortran-Python function by ```sympy.utilities.autowrap```, to have optimal inference speed, implemented in ```autofk/link.py:19 compile_homogenous()```

  * once a ```DHLink``` is constructed, it can be used to compute its corresponding homogeneous transform matrix by calling it with a joint state ```mat = link(theta)```

  the code to obtain symbolic transform matrix is implemented in ```autofk/link.py:8 homogeneous_transform()```, which takes ```a, alpha, d, theta``` and create the matrix by **intrinsically applying**:

  * ```translation-x:a```  (translation about x axis, ```distance=a```)

  * ``` rotation-x: alpha ```

  * ``` translation-z: d ```

  * ```rotation-z: theta ```


### Individual Homogeneous Transforms of KR210:

code to obtain symbolic Individual homogeneous transform matrix for each frame:

```python

import sympy as sp
from autofk.link import homogenous_transform

theta1, theta2, theta3, \
    theta4, theta5, theta6 = sp.symbols('theta1:7')

dh_table = [[0, 0, 0, theta1],
            [0.35, -sp.pi/2, 0, -sp.pi/2 + theta2],
            [1.25, 0, 0, theta3],
            [0.056, sp.pi/2, 1.5, theta4],
            [0, -sp.pi/2, 0, theta5],
            [0, sp.pi/2, 0, theta6]]

matrices = [homogenous_transform(*x) for x in dh_table]

print 'homogenous transform of link 1-3\n'
sp.pprint((matrices[0], matrices[1], matrices[2]), num_columns=100)

print '\nhomogenous transform of link 4-6\n'
sp.pprint((matrices[3], matrices[4], matrices[5]), num_columns=100)
```

the result:

![alt text][homogenous]


### Forward Kinematics Prediction:

the purpose of forward kinematics prediction is to map joint coordinates to euclidean coordinates

  * inputs: all joint states, the base_footprint frame (described in world frame). In the case of non-mobile robots the base_footprint frame is the world frame.

  * outputs: the value (described by a homogeneous transform matrix) of each DH-frame described in world frame.

  * the end-effector position and orientation can be extracted from the last DH-frame value (up to a fixed homogeneous transform)

the software implementation of this forward prediction is in:

  * ```autofk/chain.py:127 Chain::forward() ```


### Forward Kinematics Implementation, Manual v.s. Automatic Approach:

To wrap up, as we discussed above, the forward kinematics of a robot can be constructed throw steps:

  1. Read ```<joint> <origin>``` and ```<joint> <axis> ``` from URDF file. We call the frame described in ```<joint> <origin>``` **kinematics frame**. Read the joint types from tag ```<joint> <type> ``` tag.

  2. read joint connection topology (in order to know parent-child relationship between joints)

  3. construct homogeneous transformations from the readings in ```step-1```

  4. construct ```zero-pose absolute frame value``` (frame described in world frame by a homogeneous transform matrix) for each kinematics frame

  5. construct **zero pose DH-frames** by the **basic rules** and **zero-minimizing principle** described above

  6. construct initial DH-parameter table from **zero pose DH-frames**

  7. compile each symbolic transform functions from  DH-parameter table and joint types

  8. implement a forward kinematics inference function which takes joint states and foot-frame of the robot and sequentially multiplies each transform to obtain numerical value of each DH-frame (and kinematics frame). The eef frame can be obtained by multiplying a fixed homogeneous transform to the last DH-frame

  these steps can be manually implemented case by case, by reading and filling value, performing geometric analysis over schematics, constructing and selecting common normals by experience.

  Manual implementation is educational, but in practice automatic approach has following **advantages**:

  * it frees us from performing kinematics analysis repeatedly by following the same principles and rules every time.

  * it has better correctness and optimality guarantee.

  * it enables us to implement automatic inverse kinematics.

  * it scales better to larger and more complicated structure.

  In this project **I implemented the automatic forward kinematics inference**:

  1. parse and extract information from URDF file:

      - implemented in ```autofk/parser.py: 121 URDFParser```

      - parse from URDF file:
          ``` python
          from autofk.parser import URDFParser
          parser = URDFParser(path='my_good_robot.urdf')
          ```

      - parse from ros parameter server and print joint objects:
        ```python
        import rospy
        from autofk.parser import URDFParser
        urdf = rospy.get_param('robot_description')
        parser = URDFParser(data=urdf)
        for joint in parser.joints:
          print joint
          ```

  2.  construct forward kinematics:
      ```python
      from autofk.dag import KinematicsDag
      from autofk.parser import URDFParser
      parser = URDFParser(path='my_good_robot.urdf')
      dag = KinematicsDag(parser.joints)

      # accessing the first chain structure
      chain = dag.first.chain
      ```
  3. forward kinematics inference with chain structure and broadcast to ros tf:

      ```python
      from autofk.dag import KinematicsDag
      from autofk.parser import URDFParser
      import tf # to broadcast DH-frames

      br = tf.TransformBroadcaster()

      urdf = rospy.get_param('robot_description')
      parser = URDFParser(data=urdf)
      chain = parser.first.chain

      joint_states = ... # whatever joint states from ros

      # return a tuple of (link_name, frame_matrix) of each link
      frames = chain.forward(joint_states, return_dh=True)

      # get the base_link name to publish
      base_name, _ = chain.dh_base_link

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
          - the partition into DAG is implemented in ```autofk/adg.py: partition_as_serial_dag()```
          - in the case of kr210, the dag consists of three nodes: ```the entire robot arm```, ```left_gripper_finger```, and ```right_gripper_finger```.

      2. each serial structure has its own forward kinematics, represented by a ```autofk/chain.py:116 Chain``` class. The chain class perform DH frame and parameter inference task.


### Geometric Inverse Kinematics:

  * after the construction of forward kinematics, we need an inverse function maps euclidean space trajectory plan to joint state actions.

  * the overall strategy to create such inverse function is kinematics decoupling:

      * the first three joints are responsible for correctly positioning the wrist center.

      * the last three joints are responsible for the orientation of the eef.

  * we define a set of terminology:

    - ```waist```: first joint, since it controls the x-y-planar rotation of the robot list human waist

    - ```shoulder and elbow```: second and third joints, for the same anthropic reason above

    - ```wrist```: the collection of the last three joints, since it controls the orientation of the end effector like human wrist

    - ```wrist center```: the intersection of last three joint axis

    - ```H_A_B```: the homogeneous matrix of frame B described in frame A. ```A=0``` means frame described in world-frame (```base_footprint```)


### Compute Wrist Center Position:

  to obtain joint states from euclidean coordinate of eef, we first obtain the required position of the wrist center:

  * ```H_6_E```: is defined as the matrix of eef frame described in link-6's DH frame, obtained by forward kinematics above.

  * ```H_6_E``` is a fixed 4x4 matrix since eef is fixed w.r.t link-6.

  * ```H_0_E``` is the input to our problem.

  * therefore: ```H_0_6 = np.matmul(H_0_E, np.linalg.inv(H_6_E))``` as implemented in ```kuka_kinematics/geometric_ik.py:138 infer_joints()```

  * extract wrist center position: ```wc_pos = H_0_6[:3, 3]```, implemented in ```kuka_kinematics/geometric_ik.py:139 infer_joints()```


### Solve Waist:

  * in the zero-pose arm, origins ```O1, O2, O3, O4(wrist center)``` are in the same plane, we call it ```plane-2```: the plane pass through ```O2``` and perpendicular to ```z2```

  * the rotation w.r.t ```z2, z3``` will preserve this property, since these two rotation axis are perpendicular to ```plane-2```

  * therefore we can completely determine the joint state of waist by projecting wrist center to ```x-y-plane``` of the world-frame.

  * hence ```waist_state =  np.arctan2(wc_pos[1], wc_pos[0]) ``` implemented in ```kuka_kinematics/geometric_ik.py: 140```


### Solve Shoulder:

  * once the waist state is known, we next proceed to solve shoulder and elbow state.

  ![alt text][geo_ik]

  * to perform numerical analysis, first compute the coordinate of wrist center described in in ```plane-2```, this is implemented in ```kuka_kinematics/geometric_ik.py:177 infer_shoulder()```.

  * perform a partial forward kinematics inference to compute  ```frame-2``` from the results we obtained in last section.

  * describe the wrist center in ```frame-2``` by left multiplying with the inverse(transpose) of ```frame-2```'s rotation matrix followed by a translation of negative ```o1-o2``` vector. The new coordinate is denoted as ```new_wc```

  * we can now solve angle ```phi-1``` by ```phi_1 = arctan2(new_wc_y, new_wc_x)``` as implemented in ```kuka_kinematics/geometric_ik.py:184```

  * two important geometric quantities need to be obtained, the ```upper-arm-length``` and the ```lower-arm-length```. The computation is implemented in ```kuka_kinematics/geometric_ik.py:117 KukaGeometricIK::__init__()```

  * we can now solve ```phi-2``` by triangular equation:

    ```
    lower_arm**2 = upper_arm**2 + o2_wc**2 - 2cos(phi2)(upper_arm * o2_wc)
     ```

    as implemented in ```kuka_kinematics/geometric_ik.py:61```

  * the shoulder state ```phi-5``` can be readily obtained by ```phi5 = pi/2 - (phi2 + phi1)```


### Multiple Solutions in Solving Shoulder:

  * there are two solutions for shoulder state (by first ignoring joint constraint), as illustrated in in the schematics:

  ![alt text][multi_solution]

  * the strategy to attack this problem is:

      - compute all possible solutions.
      - filter the solution set by joint limit.
      - select solution that has least cumulative circular distance in joint space. (there could be a better criteria by weighting the joints, but we'll live with this one)

  * the implementation: ```kuka_kinematics/geometric_ik.py:185-189 infer_shoulder()``` and ```kuka_kinematics/geometric_ik.py:141-144 infer_joints()```.


### Solve Elbow:

  * we first perform a partial forward kinematics inference with the knowledge of waist and shoulder state we obtained so far and set the elbow state to zero.

  * the return value contains coordinate of ```O4``` if elbow state is zero, denoted as ```O4Zero```. It also contains coordinate of ```O3``` and ```Z3```.

  * we now compute the twist angle between vectors ```O3-O4Zero``` and ```O3-wc``` w.r.t ```Z3```.

  * the implementation: ```kuka_kinematics/geometric_ik.py:191 infer_elbow()```

### Install the dependencies (python-gflags and sympy):

  ```bash
  pip install python-gflags sympy
  ```

### Forward Kinematics Experiments:

  To validate the correctness of our implementation of ```autofk```, the first experiment is to broadcast the DH-frame obtained in ```tf```, and check:

  1. if the frame is correctly attached to each link, as the robot moves.

  2. if the frame satisfies the DH-rules we obtained in previous sections.

  start ```ros``` and ```gazebo``` environment, load our kr210:
  ```bash
  $ roslaunch kuka_arm forward_kinematics.launch
  ```

  after the environment is fully loaded, start broadcast to ```tf``` the DH-frames:

  ```bash
  $ cd $PROJECT_ROOT && python -m kuka_kinematics --dh_relative=False --ik_server=False --frame_type=dh
  ```

  the intial frame assignment:

  ![alt text][fk_intial]


  the frame after joint states are changed:

  ![alt text][fk_move]

  enable robot model:

  ![alt text][fk_model]

  as we can see, the correctness of forward kinematics is validated, **correctness adhere** all the rules in DH-frame selection and **move rigidly** with each link.



### Inverse Kinematics Experiments:

  we use the pick-n-place task to validate our geometric_ik implementation:

  start ros and pick-n-place task:

  ```bash
  $ roscd kuka_arm/scripts && sh safe_spawner.sh
  ```

  start our IK-server:

  ```bash
  % cd $PROJECT_ROOT && python -m kuka_kinematics --ik_server=True
  ```
  wait for the cmd prompt: **ik server started**

  in rviz, repeatedly click next and wait for the robot to complete motion command.





### Future Improvements:

  * more joint type support should be added to ```autofk```

  * velocity kinematics and dynamics support should be added to ```autofk```

  * in our implementation of kr210 geometric_ik, we can see to generate IK solution, one need to repeatedly (partially) evaluate forward kinematics with current knowledge, this implies the generic solution of inverse kinematics takes the form of **intelligently search through a tree structure of possible decisions** with **geometric prior and intuition**.

  * the ```autoik``` is under development, using neural nets and reinforcement learning to generate IK solutions for any structure generated by ```autofk```

  * richer test bed for ```autofk``` and ```autoik```: build more tasks and robot models.
