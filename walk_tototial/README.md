# 
ROS install:
http://wiki.ros.org/melodic/Installation/Ubuntu
   sudo apt install ros-melodic-catkin python-wstools python-catkin-tools

cd
mkdir -p catkin_ws/walk_tutorial/src
cd catkin_ws/walk_tutorial/
wstool init src
curl https://raw.githubusercontent.com/agent-system/lecture20201/main/walk_tutorial/.rosinstall | wstool merge -t src -
wstool update -t src
source /opt/ros/melodic/setup.bash
rosde install -y -r --from-paths src --ignore-src
catkin build -c
source ~/catkin_ws/walk_tutorial/devel/setup.bash
rtmlaunch hrpsys_ros_bridge samplerobot.launch USE_UNSTABLE_RTC:=true


another terminal
(1)
cd lecture2021/walk_tutorial
roseus walk-samplerobot-hrpsys.l

(2)
cd lecture2021/walk_tutorial
roseus walk-samplerobot-eus.l


