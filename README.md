# FarmSim

Simulated macroalgae farm in Unity.

## First-Time Setup

This project is currently being developed with Ubuntu 18.04, ROS Melodic, and Unity 2020.1.6.f1.

```bash
cd ros/src
catkin build # Assumes you have catkin-tools installed.
```

## Running Unity with ROS

```bash
# Launch to rosbridge server.
roslaunch rosbridge_server rosbridge_websocket.launch
```
