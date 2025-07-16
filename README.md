# CloCk: Closed-loop control with Core Knowledge
The purpose of this library is to showcase a framework for multi-step ahead plannig using pure input control. The navigation problem is broken down into several unique closed-loop input controllers, called Tasks. Each tasks produces a unique control behaviour (go straight, turn left/right 90 degrees) in response to a disturbance object. A supervising module, called the Configurator, can simulate sequences of Tasks  in game engine [Box2D](https://github.com/glafratta/box2d), extracts plans in the discrete and continuous domain, and queue them for execution.

## Features:
* Flexible task duration achieved through a novel attention mechanism based on the construction of simulated distal sensors

* Work in progress: purely closed-loop Task execution in the real world using said attention mechanism

* Work in progress: real-time adjustment of a threshold used to determine disturbance novelty

### Documentation

Available at [https://glafratta.github.io/bratmobile/](https://glafratta.github.io/bratmobile/)

## Hardware
The indoor robot is equipped with 
* 360 Parallax Continuous Rotation Servo motors (see [here](https://github.com/berndporr/alphabot/blob/main/alphabot.cpp) for wiring)
* A1 SLAMTEC LIDAR (see [here](https://github.com/berndporr/rplidar_rpi) for wiring)
* Raspberry Pi model 3b+

## Prerequisites
### Development packages

* G++ compiler
* CMake
* PiGPIO library
* OpenCV
* Boost
* XOrg
* LibGLU1
* Gtest

`sudo apt install g++ cmake libpigpio-dev libopencv-dev libboost-all-dev xorg-dev libglu1-mesa-dev libgtest-dev`

### Compile from source

* [LIDAR API](https://github.com/berndporr/rplidar_rpi)
* [Motors API](https://github.com/berndporr/alphabot)
* [Cpp Timer](https://github.com/berndporr/cppTimer)
* [Box2D v2.4.1](https://github.com/glafratta/box2d)
  ** if not installed automatically, go to `box2d/build` and run `sudo make install`

## Build
```
cd CloCK
sh build.sh
```

## Run
### Navigation demo (Raspberry Pi)
* `sudo ./targetless` : this program demonstrates planning over a 1m distance horizon for a control goal that is not a target location but rather an objective to drive straight for the longest time with the least amount of disturbances
* `sudo ./target`: this program (under construction) demonstrates target seeking behaviour, where the target is imaginary and located at x=1.0m, y=0m.
Run with options `0 [custom-stepDistance]`: for turning debug options off. In debug mode, LIDAR coordinates, Box2D objects and robot trajectories are dumped into the `/tmp` folder. The stepDistance is the maximum distance covered by a single task, 1.0m by default.

Only planning and execution (tracking using dead reckoning) demonstrated

WARNING! Due to the point cloud clustering algorithm used (Partition), an obstacle such as a cul-de-sac will be detected as a solid box, so take into account that representation of concave objects may be inaccurate.
### Simulation (x86 architecture)
* `test/simulations/target_sim [folder_with_scans/] [bool: is_real_time]` : runs a simulation of a planning and plan recycling scenario (not fully debugged). Args:
 - `folder_with_scans` : a folder containing 2D LIDAR scans
 - `is_real_time` : a boolean flag used to determine whether to use a timer to ensure that LIDAR callbacks are called every 200ms and motor callbacks every 100 (default=1)

### Unit tests (x86 architecture)
run `make test`

