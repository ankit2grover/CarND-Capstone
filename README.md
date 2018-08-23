## Capstone project

This is the project repo for the final project of the Udacity Self-Driving Car Nanodegree: Programming a Real Self-Driving Car. 

## Please note that this is individual submission without any team.


## Architecture Diagram

![image](docs/architecture.png)

### Native Installation

* Be sure that your workstation is running Ubuntu 16.04 Xenial Xerus or Ubuntu 14.04 Trusty Tahir. [Ubuntu downloads can be found here](https://www.ubuntu.com/download/desktop).
* If using a Virtual Machine to install Ubuntu, use the following configuration as minimum:
  * 2 CPU
  * 2 GB system memory
  * 25 GB of free hard drive space

  The Udacity provided virtual machine has ROS and Dataspeed DBW already installed, so you can skip the next two steps if you are using this.

* Follow these instructions to install ROS
  * [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) if you have Ubuntu 16.04.
  * [ROS Indigo](http://wiki.ros.org/indigo/Installation/Ubuntu) if you have Ubuntu 14.04.
* [Dataspeed DBW](https://bitbucket.org/DataspeedInc/dbw_mkz_ros)
  * Use this option to install the SDK on a workstation that already has ROS installed: [One Line SDK Install (binary)](https://bitbucket.org/DataspeedInc/dbw_mkz_ros/src/81e63fcc335d7b64139d7482017d6a97b405e250/ROS_SETUP.md?fileviewer=file-view-default)
* Download the [Udacity Simulator](https://github.com/udacity/self-driving-car-sim/releases/tag/v0.1).

### Usage

1. Clone the project repository
```bash
git clone https://github.com/tokyo-drift/capstone-project.git
```

2. Install python dependencies
```bash
cd capstone-project
pip install -r requirements.txt
```
3. Make and run styx
```bash
cd ros
catkin_make
source devel/setup.sh
roslaunch launch/styx.launch
```
4. Run the simulator

## Implementation details

### Waypoint updater

Waypoint updater publishes the next 200 waypoints ahead of the car position, with the velocity that the car needs to have at that point. Each 1/20 seconds, it does:

- Update of closest waypoint. It does a local search from current waypoint until it finds a local minimum in the distance. If the local minimum is not near (less than 20m) then it assumes it has lost track and does perform a global search on the whole waypoint list.
- Update of velocity. If there is a red ligth ahead, it updates waypoint velocities so that the car stops `~stop_distance` (*node parameter, default: 5.0 m*) meters behind the red light waypoint. Waypoint velocities before the stop point are updated considering a constant `~target_brake_accel` (*default: -1.0 m/s^2*).

Besides, the car is forced to stop at the last waypoint if either its velocity in `/base_waypoints` is set to 0 or the parameter `~force_stop_on_last_waypoint` is true.

### Drive By Wire Node

This module takes place of controlling 3 values: the steering, the throttle and the brake.

#### Steering 
Steering is handled by a combination of predictive steering and corrective steering. Predictive Steering is implemented using the provided `YawController` class. Corrective steering is computed by calculating the cross-track-error which is then passed to a linear PID which returns the corrective steering angle. These are added together to give the final steering angle

#### Throttle
Throttle is controlled by a linear PID by passing in the velocity error(difference between the current velocity and the proposed velocity)


#### Brake
If a negative value is returned by the throttle PID, it means that the car needs to decelerate by braking. The braking torque is calculated by the formula `(vehicle_mass + fuel_capacity * GAS_DENSITY) * wheel_radius * deceleration`


### Traffic light detector

Traffic light detection is based on pre-trained on the COCO dataset model [ssd_mobilenet_v1_coco](http://download.tensorflow.org/models/object_detection/ssd_mobilenet_v1_coco_11_06_2017.tar.gz) from https://github.com/tensorflow/models/blob/master/research/object_detection/g3doc/detection_model_zoo.md

The traffic light classification model is [a SqueezeNet model](https://arxiv.org/abs/1602.07360) trained on some real life and simulator traffic light images in project.

The ROS traffic light detector is implemented in node `tl_detector` in classes `TLDetector` and `TLClassifier`. `TLDetector` is responsible for finding a nearest traffic light position and calls `TLClassifier.get_classification` with the current camera image. `TLClassifier` first uses the SSD MobileNet model to detect a traffic light bounding box with the highest confidence. If the bounding box is found a cropped traffic light image is scaled to a 32x32 image and the SqueezeNet model infers the traffic light color (red, amber, green). If at least 3 last images were classified as red then `TLDetector` publishes the traffic light waypoint index in the `/traffic_waypoint` topic.
