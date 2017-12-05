# Demo of RCV in ROS/GAZEBO

This is a simulation of a [KTH Research Concept Vehicle (RCV)](https://www.itrl.kth.se/research/itrl-labs/rcv-1.476469) in [gazebo 8](http://gazebosim.org) with sensor data being published using [ROS kinetic](http://wiki.ros.org/kinetic/Installation).

The RCV's wheel torques, curvature and crabbing angle are controlled by publishing a ROS message. A ROS node allows controlling the RCV to maintain at a predefined speed or follow a path.

# Video + Pictures

Video is not available yet. Will be released in two weeks.

[!RCV Image](https://github.com/txzhao/car_demo/tree/master/pic/rcv.png)

# Requirements

This demo has been tested on Ubuntu Xenial (16.04)

* An X server
* [Docker](https://www.docker.com/get-docker)
* [nvidia-docker](https://github.com/NVIDIA/nvidia-docker/wiki/Installation)

# Recommended

* A joystick
* A joystick driver which creates links to `/dev/input/js0` or `/dev/input/js1`

# Building

First clone the repo, then run the script `build_demo.bash`.
It builds a docker image with the local source code inside.

```
$ cd car_demo
$ ./build_demo.bash
```

# Running

Connect a game controller to your PC.
Use the script `run_demo.bash` to run the demo.

```
$ ./run_demo.bash
```
An [RVIZ](http://wiki.ros.org/rviz) window will open showing the car and sensor output.
A gazebo window will appear showing the simulation.
Either use the controller to drive the prius around the world, or click on the gazebo window and use the `WASD` keys to drive the car.

If using a Logitech F710 controller:

* Make sure the MODE status light is off
* Set the swtich to XInput mode
* The right stick controls throttle and brake
* The left stick controls steering
* Y puts the car into DRIVE
* A puts the car into REVERSE
* B puts the car into NEUTRAL
