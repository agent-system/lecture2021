# Create URDF model and convert to EusLisp model

## PR2
https://github.com/jsk-ros-pkg/jsk_pr2eus/raw/master/pr2eus/pr2.l


## PAL/Tiago

```
$ sudo apt install ros-${ROS_DISTRO}-jsk-model-tools

$ mkdir tiago_ws
$ cd tiago_ws
$ wstool init src https://raw.githubusercontent.com/pal-robotics/tiago_tutorials/kinetic-devel/tiago_public-melodic.rosinstall

$ catkin build teago_description

$ source tiago_ws/devel/setup.bash

$ roslaunch tiago_description show.launch  ## -> you can see the model in rviz
 ## at another terminal while running rviz
$ rosparam get -p /robot_description | sed -e 's@|@@' > tiago.urdf

$ rosrun euscollada collada2eus -I tiago.urdf -C tiago.yaml -O tiago.l
  ## tiago.yaml is this file

$ roseus tiago.l ## you get tiago robot as EusLisp model.
roseus $ (setq *robot* (tiago))
roseus $ (objects (list *robot*))
```

## Universal Robot and Robotiq Hand

```
$ sudo apt install ros-${ROS_DISTRO}-jsk-model-tools

$ mkdir /universal_ws
$ cd /universal_ws

$ wstool init src
$ wstool set -y -t src universal_robot https://github.com/ros-industrial/universal_robot.git --git
$ wstool set -y -t src robotiq https://github.com/ros-industrial/robotiq.git --git
$ wstool update -t src

$ catkin init
$ catkin build ur_e_description robotiq_3f_gripper_visualization

$ source /universal_ws/devel/setup.bash

$ roslaunch ur_e_description view_ur5e.launch  ## -> you can select ur3e, ur10e insted of ur5e
 ## at another terminal while running rviz
$ rosparam get -p /robot_description | sed -e 's@|@@' > ur5e.urdf

$ rosrun euscollada collada2eus -I ur5e.urdf -C ur5e.yaml -O ur5e.l

$ roseus ur5e.l
roseus $ (setq *robot* (ur5e))
roseus $ (objects (list *robot*))

$ ROS_NAMESPACE=robotiq roslaunch robotiq_3f_gripper_visualization robotiq_gripper_upload.launch
$ rosparam get -p /robotiq/robot_description | sed -e 's@|@@' > robotiq_3f.urdf
$ rosrun euscollada collada2eus -I robotiq_3f.urdf -C robotiq_3f.yaml -O robotiq_3f.l
```

## Kuka/youbot

```
$ sudo apt install ros-${ROS_DISTRO}-jsk-model-tools

$ mkdir /youbot_ws
$ cd /youbot_ws

$ wstool init src
$ set -y -t src youbot_description https://github.com/mas-group/youbot_description.git --git
$ wstool update -t src

$ catkin init
$ catkin build youbot_description

$ source /youbot_ws/devel/setup.bash

$ roscd youbot_description
$ rosrun xacro xacro --inorder robots/youbot.urdf.xacro > youbot.urdf

$ rosrun euscollada collada2eus -I youbot.urdf -C youbot.yaml -O youbot.l

$ roseus youbot.l
roseus $ (setq *robot* (youbot))
roseus $ (objects (list *robot*))
```

## BostonDynamics/Spot

```
$ sudo apt install ros-${ROS_DISTRO}-jsk-model-tools

$ mkdir /spot_ws
$ cd /spot_ws

$ wstool init src
$ set -y -t src spot_ros https://github.com/clearpathrobotics/spot_ros.git --git
$ wstool update -t src

$ catkin init
$ catkin build spot_description

$ source /spot_ws/devel/setup.bash
$ roslaunch spot_description description.launch
$ rosparam get -p /robot_description | sed -e 's@|@@' > spot.urdf
$ rosrun euscollada collada2eus -I spot.urdf -C spot.yaml -O spot.l

$ roseus spot.l
roseus $ (setq *robot* (spot))
roseus $ (objects (list *robot*))
```
