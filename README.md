The purpose of this library is to showcase a framework for multi-step ahead planning using pure input control (see [Braitenberg vehicles](https://en.wikipedia.org/wiki/Braitenberg_vehicle)). The navigation problem is broken down into several unique closed-loop input controllers, called Tasks. Each Task produces a unique control behaviour (go straight, turn left/right 90 degrees) in response to a [Disturbance](https://en.wikipedia.org/wiki/Errors_and_residuals) (an obstacle or target), which determines a Task's duration. A supervising module, called the Configurator, can simulate sequences of Tasks at runtime in game engine [Box2D](https://github.com/glafratta/box2d), retain their outcomes in a cognitive map, which can be searched to extract plans. The physics simulation represents the robot's [Core Knowledge](https://www.harvardlds.org/wp-content/uploads/2017/01/SpelkeKinzler07-1.pdf) (Spelke, 2007).

## Features:
* Hybrid state-space representation where states are Tasks focused on a specific object
  
* Mapless navigation: robot is ready to go as is, no need for global maps/SLAM

* Plans are made up of multiple Tasks (object-focused instructions), not trajectories

* On-the-fly replanning

* `fastdds` folder: provides classes to publish Task data to a Qt window 

### Documentation

Available at [https://glafratta.github.io/bratmobile/](https://glafratta.github.io/bratmobile/)

### Publications

Giulia Lafratta, Bernd Porr, Christopher Chandler, Alice Miller; Closed-Loop Multistep Planning. Neural Computation 2025; 37 (7): 1288–1319. doi: [https://doi.org/10.1162/neco_a_01761](https://doi.org/10.1162/neco_a_01761)

## Hardware
The indoor robot is equipped with 
* Raspberry Pi model 3b+
* 360 Parallax Continuous Rotation Servo motors (see [here](https://github.com/berndporr/alphabot/blob/main/alphabot.cpp) for wiring)
* A1 SLAMTEC LIDAR (see [here](https://github.com/berndporr/rplidar_rpi) for wiring)


## Prerequisites
### Development packages

`sudo apt install g++ cmake libpigpio-dev libopencv-dev libboost-all-dev xorg-dev libglu1-mesa-dev libgtest-dev`

### Libraries to compile from source

* [LIDAR API](https://github.com/berndporr/rplidar_rpi)
* [Motors API](https://github.com/berndporr/alphabot)
* [Cpp Timer](https://github.com/berndporr/cppTimer)
* [Box2D v2.4.1](https://github.com/glafratta/box2d)
  ** if not installed automatically, go to `box2d/build` and run `sudo make install`

## Clone 

`git clone git@github.com:glafratta/bratmobile.git`

## Build
```
cd bratmobile
cmake .  
make
sudo make install
```

## Run
### Navigation demo (Raspberry Pi)
Demo prefixes:

* `brat2*` : Multi-step planning with fixed discretisation of Tasks with DEFAULT actions
* `brat3*` : Multi-step planning with fixed-size state split (of states ending in collision) and attention window to guide optimal obstacle avoidance when a goal is present

Demos:

* `sudo ./*targetless` : these programs demonstrates planning over a 1m distance horizon for a control goal that is not a target location but rather an objective to drive straight for the longest time with the least amount of disturbances
* `sudo ./*target`: these program demonstrates target seeking behaviour, where the target is imaginary and located at x=1.0m, y=0m.

### Unit tests 
`ctest`

run `make test`

