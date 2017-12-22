# Demo of RCV in ROS/GAZEBO

This is a simulation of a [KTH Research Concept Vehicle (RCV)](https://www.itrl.kth.se/research/itrl-labs/rcv-1.476469) in [gazebo 8](http://gazebosim.org) with sensor data being published using [ROS kinetic](http://wiki.ros.org/kinetic/Installation). The RCV's four-wheel torques, curvature and crabbing angle are controlled by publishing a ROS message. A ROS node allows controlling the RCV to follow a predefined path.

This repo also serves as the RCV simulator for the project course [EL2425 Automatic Control, Project Course](https://www.kth.se/social/course/EL2425/) at KTH. To get more information about the project and how to control the real RCV, please go to this [repo](https://github.com/txzhao/Model-Control-RCV).

#### Detailed origins

- The skeleton is a modified version of [osrf/car_demo](https://github.com/osrf/car_demo) and [ecward/car_demo](https://github.com/ecward/car_demo);
- The PID controller is taken from [ivmech/ivPID](https://github.com/ivmech/ivPID);
- The MPC controller is a rewrite of Goncalo's Matlab implementation in his [master thesis](https://kth.diva-portal.org/smash/get/diva2:1043944/FULLTEXT01.pdf);
- The QP solver comes from [CVXOPT](https://github.com/cvxopt/cvxopt).

#### Graph of ROS nodes

![Rosgraph](https://github.com/txzhao/car_demo/blob/master/pic/rosgraph.png)

## Why this repo?

[Integrated Transport Research Lab (ITRL)](https://www.itrl.kth.se/) aims to perform automatic control algorithms to the RCV and conduct experiments to test its performance. Since testing on the real RCV takes quite a lot of time and efforts, a RCV simulator is highly needed for pre-testing purpose. 

Given the demands, this repo modifies a similar open-source work [car_demo](https://github.com/osrf/car_demo) and partially changes the Toyata car's inner dynamics to match with that of RCV, and then performs several automatic controllers to control the car. Centralized configuration and evaluation tools are also developed to improve user experience.

## Changes from previous work

#### [RCVPlugin.cc](https://github.com/txzhao/car_demo/blob/master/car_demo/plugins/RCVPlugin.cc)
- Inner dynamics (weight, dimension etc.): Toyota car -> RCV;
- Interface: throttle, brake, shift_gear, steer -> four-wheel torques, curvature and crabbing angle.
#### [move.py](https://github.com/txzhao/car_demo/blob/master/car_demo/src/move.py)
- Joystick control (manual) -> PI control (velocity) + pure pursuit / MPC (path follow).
#### [liveplot.py](https://github.com/txzhao/car_demo/blob/master/car_demo/src/liveplot.py)
- State visualization in real time.

## Video + Pictures

The video of simulaton and real RCV control test could be found [here](https://www.youtube.com/watch?v=nw0xhZjIuw8).

#### Screenshot of RCV model

<p align="center">
<img src="https://github.com/txzhao/car_demo/blob/master/pic/rcv.png" width="800"/>
</p>

## Requirements

- Ubuntu Xenial (16.04)
- ROS Kinetic Kame
- Gazebo 8.0.0
- Dependencies: ros-kinetic-desktop-full, ros-kinetic-fake-localization, ros-kinetic-joy
- QP solver: CVXOPT

For detailed instructions of installation, please turn to this [text file](https://github.com/txzhao/car_demo/blob/master/install_instructions.txt) for help (ubuntu system).

## How to run

First clone this repo into the src folder of your catkin workspace. Then in the toplevel catkin workspace folder, run the following command in the terminal

```
$ catkin_make
$ source devel/setup.sh
$ roslaunch car_demo rcv_sim.launch visual:=true
```

Now, the simulator along with a simple live plot which records RCV's current position and predefined path should show up. If you want to disable the live plot, simply replace the last command line with ```roslaunch car_demo rcv_sim.launch```.

To run the controller, open another terminal window and run ```rosrun car_demo move.py```.

### Configurations

When started, both of the simulator and live plot will first read the centralized configurations in [config.ini](https://github.com/txzhao/car_demo/blob/master/car_demo/src/configs/config.ini). This file contains all the related parameters of the simulator, live plot and controllers. You can try tuning these parameters to see how they may influence the performances in simulation.

### Logs

The states of RCV are stored in a .csv file in [/logs](https://github.com/txzhao/car_demo/tree/master/car_demo/src/logs). You can retrieve and use them to further verify your controller.

## Results

#### Straight Line + Pure Pursuit

<p align="center">
<img src="https://github.com/txzhao/car_demo/blob/master/pic/line_4x.gif" width="800"/>
</p>

#### Zigzag + Pure Pursuit

<p align="center">
<img src="https://github.com/txzhao/car_demo/blob/master/pic/zigzag_3x.gif" width="800"/>
</p>

#### Circle + Pure Pursuit

<p align="center">
<img src="https://github.com/txzhao/car_demo/blob/master/pic/circle_4x.gif" width="800"/>
</p>
