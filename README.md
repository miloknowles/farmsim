# FarmSim

A simulated macroalgae farm in Unity3D.

![Banner image](/resources/images/banner.png?raw=true)

Check out the [design document](https://docs.google.com/document/d/1a0eiJ9ga0okX0kW2TVdClzvAI5Pwo11VVDSLBfVIKrc/edit?usp=sharing) for details on naming, coordinate frames, ROS topics, etc.

## Requirements

This project is currently being developed with:
- Ubuntu 18.04
- ROS Melodic
- Unity 2020.1.6.f1

## First-Time Setup

```bash
cd ros/src
catkin build # Assumes you have catkin-tools installed.
```

## Connecting Unity3D to ROS

We use the [`ROSBridgeLib`](https://github.com/MathiasCiarlo/ROSBridgeLib) library to connect Unity3D to ROS.

```bash
# Launch to rosbridge server before playing the Unity simulation.
source ros/devel/setup.bash
roslaunch rosbridge_server rosbridge_websocket.launch

# Publish a thrust command.
rostopic pub -r 1 /auv/controls/trident_thrust/ control/TridentThrust '{F_lt: 1.0, F_rt: 1.0, F_ct: -1.0}'
```

## Troubleshooting

Sometimes you'll get stuck in an error loop on launch about the editor layout. It can be fixed by copying an existing `CurrentLayout-default.dwlt` into the project:
```bash
# Don't modify this file!
cp ~/resources/CurrentLayout-default.dwlt Library/
```
