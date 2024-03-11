# This version Enables demonstration of grasping and navigation tasks.
## The pybullet environment loading part has been modified:The relevant code is in the src/ur5_pybullet_ros/script/env.py folder. (You can copy the relevant code to env.py of version2.0 pkg to implement the loading function)
## How to load ur URDF file:
### 1.Config File
The config file is located at src/ur5_pybullet_ros/script/config/config.yaml
```sh
use_config: True  # determines whether the config file is enabled(False means it will not affect the simulation)

robot:
  initial_x: 0.0  # initial robot position x
  initial_y: 0.0  # initial robot position y

obstacles:  # the object to be loaded into the simulation
- name: "cube.urdf" # name of the urdf file
  pos: [0, 0, 0.5] # position to load the object
- name: "cylinder.urdf"
  pos: [0, 1, 0]
```
### 2.Please put the object urdf file into this folder:
The directory of obstacles is src/ur5_pybullet_ros/script/urdf/obstacles.
###  
###  
### python3-empy
```sh
sudo apt-get install python3-empy
```

### opencv_python scipy open3d
```sh
sudo apt-get install opencv_python
sudo apt-get install scipy
sudo apt-get install open3d
```

## Build
 in the ros workspace, run the following command in the terminal:
 ```sh
catkin build
 ```

## Setup
 Run the following command in the terminal to edit your ~/.bashrc
``` sh
gedit ~/.bashrc
```
add the following command at the end of your ~/.bashrc file.
``` sh  
source [the absolute path of your ros workspace]/devel/setup.bash
```
where the [the absolute path of your ros workspace] is the the absolute path of your ros workspace. You can get this path by running command *pwd* in your terminal open at the folder of the ros workspace.
## Run
### Pybullet with ROS and Moveit simulation
Use the following command to launch the simulation environment of the mobile UR5 with Moveit robot arm controller.
```sh
roslaunch ur5_pybullet_ros ur5_pybullet_moveit.launch
```
Then, the simulation environment is launched, including a pybullet GUI and an Rviz window. You can click 2D Nav Goal in rviz panel, and click on the map to select a navigation target for the robot. Then, the robot will navigate to the given position, while constructing map. You can also drag the robot arm target pose to set the goal pose of the robot arm, then click Plan & Execute to control the robot arm.

###  Grasping and navigating Demo
After you launch Pybullet with ROS and Moveit simulation, Please open another terminal , use the following command to launch the task controller for the demo demonstration.
```sh
rosrun ur5_pybullet_ros task_controller.py
```
Then, the task controller will automatically send navigation and arm commands to the robot, controlling it to finish the demo task. The robot will navigate to the desk, pick up the red ball, and drop it into the tray located in the other side of the room. A dynamic obstacle is also added to show the capability of the robot to navigate through dynamic environment. Then, the robot will pick up the green ball, and put it back into the other tray near where it started.
