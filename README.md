# Engine for MuJoCo in Python (Mac/Linux)

Interactive engine to use with the official Python bindings for MuJoCo 2.2.x

+ Develop along with [uwarl-mujoco-python-viewer](https://github.com/UW-Advanced-Robotics-Lab/uwarl-mujoco-python-viewer)
+ A more efficient, robust and thread safe viewer with native MuJoCo 2.2.x with glfw
+ Works with Mac M1 with MuJoCo 2.2.x

## Tested OS Platform:
- Mac M1
- Ubuntu 18.04

## Install
```sh
$ git clone https://github.com/UW-Advanced-Robotics-Lab/uwarl-mujoco-python-engine
$ cd uwarl-mujoco-python-engine
$ pip install -e .
```

## Contents
- Engine Class
- Control command processing Class
    - PID effort control for Summit base velocity
    - Effort control for WAM joint position
- State publisher Class
    - Joint States
    - Link States

## Demo (WIP):
- https://github.com/UW-Advanced-Robotics-Lab/simulation-mujoco-summit-wam