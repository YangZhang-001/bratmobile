The purpose of this library is to showcase a framework for multi-step ahead planning using pure input control (see [Braitenberg vehicles](https://en.wikipedia.org/wiki/Braitenberg_vehicle)). The navigation problem is broken down into several unique closed-loop input controllers, called Tasks. Each Task produces a unique control behaviour (go straight, turn left/right 90 degrees) in response to a [Disturbance](https://en.wikipedia.org/wiki/Errors_and_residuals) (an obstacle or target), which determines a Task's duration. A supervising module, called the Configurator, can simulate sequences of Tasks  in game engine [Box2D](https://github.com/glafratta/box2d), retain their outcomes in a cognitive map, which can be searched to extract plans. The physics simulation represents the robot's [Core Knowledge](https://www.harvardlds.org/wp-content/uploads/2017/01/SpelkeKinzler07-1.pdf) (Spelke, 2007).

## Features:
* Completely on-the-fly, instantaneous construction of an abstract cognitive map of the environment, represented as a tree of tasks

* Hybrid state-space representation with flexible discretization

* Use of an attention window (simulated distal sensor) to define the scope of Tasks

* Mapless navigation: robot is ready to go as is, no need for global sensors

* Causal reasoning on the naive robot

* Plans represents set of instructions, not trajectories: no solvers required, completely closed-loop and object oriented

## Work in progress

* Closed-loop Task execution with goal-directed behaviour (driving towards target)

* Debugging plan checking and recycling

* Adaptive thresholding of acceptable noise in state observations

### Documentation

Available at [https://glafratta.github.io/bratmobile/](https://glafratta.github.io/bratmobile/)

### Publications

Giulia Lafratta, Bernd Porr, Christopher Chandler, Alice Miller; Closed-Loop Multistep Planning. Neural Computation 2025; 37 (7): 1288–1319. doi: [https://doi.org/10.1162/neco_a_01761](https://doi.org/10.1162/neco_a_01761)

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

