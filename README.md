# FarmSim

A simulated aquaculture farm in Unity3D.

![Banner image](/resources/images/banner2.png?raw=true)

NOTE(milo): Design doc not updated recently...
Check out the [design document](https://docs.google.com/document/d/1a0eiJ9ga0okX0kW2TVdClzvAI5Pwo11VVDSLBfVIKrc/edit?usp=sharing) for details on naming, coordinate frames, ROS topics, etc.

## Requirements

This project is currently being developed with:
- Ubuntu 18.04
- ROS Melodic
- Unity 2020.1.6.f1
- Our fork of LCM

## First-Time Setup

## Troubleshooting

### Launch Error Loop

Sometimes you'll get stuck in an error loop on launch about the editor layout. It can be fixed by copying an existing `CurrentLayout-default.dwlt` into the project:
```bash
# Don't modify this file!
cp ~/resources/CurrentLayout-default.dwlt Library/
```

### LCM Crashes Editor / Makes Simulation Hang in Play Mode

Check `~/.config/unity3d/Editor.log` for the crash.
