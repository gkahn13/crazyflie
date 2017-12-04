# CRAZYFLIE ROS PACKAGE

## Overview
--------------
This package allows for a swarm of Crazyflies to be controlled by a Ground Station computer. The basic idea here is that each Crazyflie (separated by ID) can be controlled separately through its own command and motion topics, and the data and images any Crazyflie publishes are all received by the Ground Station on only two topics. Instantiating a Crazyflie node should be done via the Ground Station. For now that means specifying an input file that holds the key value pairs of Crazyflie ID and Radio URI. We are currently looking into ways to use only one Radio URI to control multiple Crazyflies, which looks possible (see here: https://forum.bitcraze.io/viewtopic.php?t=624).

## Installation
--------------
Right now there's nothing to install ;). These are just basic files, no pkg has actually been created or tested yet.


## Functionality
--------------
###### Crazyflie {id}:
* subscribes to cf/{id}/command and cf/{id}/motion
* publishes to cf/data and cf/image


###### Controller {id}:
* subscribes to cf/{id}/data and cf/{id}/image
* publishes to cf/{id}/command and cf/{id}/motion


## ROS Layout
--------------
#### NODES
* cf_node.py
* controller_node.py


#### TOPICS
* /cf/{id}/images
* /cf/{id}/data
* /cf/{id}/motion
* /cf/{id}/command
  

#### MESSAGES

<!-- ###### CFImage
* uint16 ID
* sensor_msgs/Image fpv_image
 -->
###### CFData
* uint16 ID
* float accel_x
* float accel_y
* float accel_z

###### CFMotion
* float vel_x
* float vel_y
* float altitude

###### CFCommand
* EMERGENCY-STOP = 0, LAND = 1, TAKEOFF = 2
* int cmd


