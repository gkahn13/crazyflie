# CRAZYFLIE ROS PACKAGE

## Overview
--------------
This package allows for a swarm of Crazyflies to be controlled by a Ground Station computer. The basic idea here is that each Crazyflie (separated by ID) can be controlled separately through its own command and motion topics, and the data and images any Crazyflie publishes are all received by a Controller node. Instantiating a Crazyflie node should be done via the launch file cf.launch. You must have parameters 'id' and 'uri' for each launch file call. We are currently looking into ways to use only one Radio URI to control multiple Crazyflies, which looks possible. (see here: https://forum.bitcraze.io/viewtopic.php?t=624).

## Installation
--------------
1. Setup ROS Kinetic, and setup your ROS Workspace with catkin_ws in your ~/ path.
2. Make sure to have OpenCV3 installed for python 2 (ROS works better with OpenCV in python 2).
3. In python 3, you must pip install the cflib classes from this repository: https://github.com/bitcraze/crazyflie-lib-python. If you are on linux, make sure to also follow the "Setting udev permissions" section.
3. Clone this repository to your ~/catkin_ws/src/
4. cd ~/catkin_ws
5. catkin_make
6. source ~/catkin_ws/devel/setup.bash
7. Run the desired launch files in a python 3 environment


## Functionality
--------------
###### Crazyflie {id}:
* subscribes to cf/{id}/command and cf/{id}/motion
* publishes to cf/{id}/data, cf/{id}/pose, cf/{id}/twist, cf/{id}/imu, cf/{id}/image, cf/{id}/image_raw, cf/{id}/image/compressed


###### Controller {id}:
* subscribes to cf/{id}/data and cf/{id}/image
* publishes to cf/{id}/command, cf/{id}/motion, cf/{id}/coll


## ROS Layout
--------------
#### NODES
* cf_node.py
* controller_node.py


#### TOPICS
* /cf/{id}/coll
	* std_msgs/Bool
	* Collision indicator
* /cf/{id}/command
	* CFCommand
	* takeoff, land, estop commands
* /cf/{id}/data
	* CFData
	* All Crazyflie data at 80Hz, in CF frame of reference
* /cf/{id}/image
	* sensor_msgs/Image or CompressedImage (toggle)
	* FPV image, 720x480
* /cf/{id}/image/compressed
	* sensor_msgs/CompressedImage
	* FPV image, 720x480
* /cf/{id}/image_raw
	* sensor_msgs/Image
	* FPV image, 720x480
* /cf/{id}/imu
	* sensor_msgs/Imu
	* Imu messages in ROS frame of reference
* /cf/{id}/motion
	* CFMotion
	* Velocity and altitude setter topic
* /cf/{id}/pose
	* geometry_msgs/PoseStamped
	* Position and Orientation estimates in ROS frame of reference
* /cf/{id}/twist
	* geometry_msgs/TwistStamped
	* Angular and Linear velocity estimates in ROS frame of reference
 

#### MESSAGES

###### CFData
* Header stamp
* uint16 ID
* float32 accel_x
* float32 accel_y
* float32 accel_z
* float32 v_batt
* float64 alt
* float32 gyro_x
* float32 gyro_y
* float32 gyro_z
* float32 yaw
* float32 pitch
* float32 roll
* float32 kalman_vx
* float32 kalman_vy
* float32 pos_x
* float32 pos_y
* float32 pos_z
* float32 magx
* float32 magy
* float32 magz

###### CFMotion
* float32 x
* float32 y
* float32 yaw
* float32 dz
* bool is_flow_motion


###### CFCommand
* EMERGENCY-STOP = 0, LAND = 1, TAKEOFF = 2
* int cmd

#### CONVENTIONS / FRAME OF REFERENCE
CFData is entirely in CF Frame of Reference, while any Pose/Twist/Imu data is entirely in ROS Frame of reference.

- **CF RPY Conventions:** yaw counter clockwise is positive Z, pitch up is positive Y, roll right is positive X
- **CF Position Conventions:** X(forward), Y(left), Z(up)

- **ROS RPY Conventions:** yaw counter clockwise is positive Z, pitch up is positive X, roll right is positive Y
- **ROS Position Conventions:** X(right), Y(forward), Z(up)


## Launch Files and Scripts
-------------
#### Launch Files
* cf.launch: Launches the cf\_node, the default controller node, and the camera
* joy.launch: Launches the joystick and the joystick controller nodes.
* joycf.launch: This is the main launch file to start the cf\_node, the joystick controller node, the joystick node, and the camera node.
	* ARGUMENTS
		* id
		* cam_id
		* uri
		* use_flow
		* joy_topic
		* data_only
		* use_joy

#### Scripts
* figure8.py: Moves CF in a figure 8.
* hover\_end\_bat.py: uses altitude and optical flow position control to hover until the battery dips below a certain voltage.
* key_control.py: keyboard control of crazyflie (forward, back, left, right) example



