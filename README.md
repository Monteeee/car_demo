# Demo of RCV in ROS/GAZEBO

This is a simulation of a [KTH Research Concept Vehicle (RCV)](https://www.itrl.kth.se/research/itrl-labs/rcv-1.476469) in [gazebo 8](http://gazebosim.org) with sensor data being published using [ROS kinetic](http://wiki.ros.org/kinetic/Installation). The RCV's four-wheel torques, curvature and crabbing angle are controlled by publishing a ROS message. A ROS node allows controlling the RCV to follow a predefined path.

This repo also serves as the RCV simulator for the project course [EL2425 Automatic Control, Project Course](https://www.kth.se/social/course/EL2425/) at KTH. To get more information about the project and how to control the real RCV, please go to this [repo](https://github.com/txzhao/Model-Control-RCV).

#### Graph of ROS nodes

![Rosgraph](https://github.com/txzhao/car_demo/blob/master/pic/rosgraph.png)

## Video + Pictures

Video is not available yet. Will be released in two weeks.

![RCV Image](https://github.com/txzhao/car_demo/blob/master/pic/rcv.png)

## Requirements

- Ubuntu Xenial (16.04)
- ROS Kinetic Kame
- Gazebo 8.0.0 

## How to Run

First clone this repo into the src folder of your catkin workspace. Then in the toplevel catkin workspace folder, run the following command in the terminal

```
$ catkin_make
$ source devel/setup.sh
$ roslaunch car_demo rcv_sim.launch visual:=true
```

Now, the simulator along with a simple live plot which records RCV's current position and predefined path should show up. If you want to disable the live plot, simply replace the last command line with ```roslaunch car_demo rcv_sim.launch```.

To run the controller, open another terminal window and run ```rosrun car_demo move.py```.

## Configurations

When started, both of the simulator and live plot will read the configuration in [config.ini](https://github.com/txzhao/car_demo/blob/master/car_demo/src/configs/config.ini). This file contains all the related parameters of the simulator, live plot and controllers. By simply changing them, different control performances could be achieved.

## Logs

The states of RCV are stored in a .csv file in [/logs](https://github.com/txzhao/car_demo/tree/master/car_demo/src/logs). You can retrieve them and use them to further verify your controller.
