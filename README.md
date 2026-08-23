# Bratmobile

![alt tag](brat-overtaking.png)

The navigation problem is broken down into several unique closed-loop input controllers, called Tasks. Each Task produces a unique control behaviour (go straight, turn left/right 90 degrees) in response to a [Disturbance](https://en.wikipedia.org/wiki/Errors_and_residuals), which determines a Task's duration. A supervising module, called the Configurator, can simulate sequences of Tasks at runtime in game engine [Box2D](https://github.com/glafratta/box2d), retain their outcomes in a cognitive map, which can be searched to extract plans. The physics simulation represents the robot's [Core Knowledge](https://www.harvardlds.org/wp-content/uploads/2017/01/SpelkeKinzler07-1.pdf) (Spelke, 2007).

Note this is a fork of the orig repo: https://github.com/glafratta/bratmobile

## Features:
* Hybrid state-space representation where states are Tasks focused on a specific object
  
* Landmark based navigation: robot is ready to go as is, no need for global maps/SLAM

* Plans are made up of multiple Tasks (object-focused instructions), not trajectories

* On-the-fly replanning

* `fastdds` folder: provides classes to publish Task data to a Qt window 

* Optional ROCK 5B+ TargetLoc-to-navigation integration using camera-guided RPLIDAR target localisation

### Publications

Giulia Lafratta, Bernd Porr, Christopher Chandler, Alice Miller; Closed-Loop Multistep Planning. Neural Computation 2025; 37 (7): 1288–1319. doi: [https://doi.org/10.1162/neco_a_01761](https://doi.org/10.1162/neco_a_01761), [Preprint](https://arxiv.org/pdf/2402.15384) and [Final publication on Glasgow University's repository](https://eprints.gla.ac.uk/348892/).

## Hardware
The indoor robot is equipped with 
* Raspberry Pi model 5
* [Zetabot with 360 Parallax Continuous Rotation Servo motors and stereo cameras](https://github.com/berndporr/zetabot))
* [C1 SLAMTEC LIDAR](https://github.com/berndporr/c1lidar)

The optional ROCK 5B+ integration has been validated with:
* Radxa ROCK 5B+
* two Raspberry Pi Camera V2.1 modules
* C1 SLAMTEC LIDAR

## Prerequisites
### Development packages

```
sudo apt install g++ cmake libopencv-dev libboost-all-dev xorg-dev libglu1-mesa-dev libgtest-dev xauth x11-apps xfonts-base
```

For the ROCK 5 camera/navigation path also install:

```bash
sudo apt install pkg-config libbox2d-dev libgpiod-dev libv4l-dev v4l-utils
```

For the standalone TargetLoc viewer also install Qt6 and QCustomPlot development packages:

```bash
sudo apt install qt6-base-dev libqcustomplot-dev
```

### Libraries to compile from source

* [C1 LIDAR API](https://github.com/berndporr/c1lidar)
* [Zetabot API](https://github.com/berndporr/zetabot)
* [Cpp Timer](https://github.com/berndporr/cppTimer)
* [libcamera2opencv](https://github.com/berndporr/libcamera2opencv)

### Install powersave service

The rpi5 draws too much current under load to run off the battery so we need to enable powersave:

sudo cp powersave.service /etc/systemd/system
sudo systemctl enable --now powersave.service


## Build

```
cd bratmobile
cmake .
make
```

## Run
### Navigation demo (Raspberry Pi)
Demo prefixes:

* `brat2*` : Multi-step planning with fixed discretisation of Tasks with DEFAULT actions
* `brat3*` : Multi-step planning with fixed-size state split (of states ending in collision) and attention window to guide optimal obstacle avoidance when a goal is present

Demos:

* `./*targetless` : these programs demonstrates planning over a 1m distance horizon for a control goal that is not a target location but rather an objective to drive straight for the longest time with the least amount of disturbances
* `./*target`: these program demonstrates target seeking behaviour, where the target is imaginary and located at x=1.0m, y=0m.

### Unit tests 
`ctest`

run `make test`

---

## ROCK 5B+ TargetLoc navigation

The ROCK 5 extension connects the existing TargetLoc perception code to
`brat3_target` and the existing brat3 planner:

```text
QR detection
→ camera direction cue
→ RPLIDAR target association
→ calibrated TargetLoc coordinate
→ stable target lock
→ brat3 navigation
→ ROCK 5 wheel output
```

The final metric TargetLoc coordinate is produced by the **camera-guided
calibrated LiDAR path**. Stereo disparity is retained for diagnostics and is
not used as a stereo-LiDAR weighted final coordinate.

### Device check

The validated ROCK 5 setup used:

```text
Cameras: /dev/video23 and /dev/video32
RPLIDAR: /dev/ttyS2
```

Check the devices before running a demo:

```bash
ls /dev/video23 /dev/video32
ls -l /dev/ttyS2
```

Video device numbers can differ on another installation. If access is denied,
check the user's permissions for the camera and serial devices.

### Important CMake options

| Option | Purpose |
|---|---|
| `TARGETLOC_USE_ROCK5_V4L_CAMERA=ON` | use the ROCK 5 V4L/OpenCV camera backend |
| `BRAT_BUILD_TARGETLOC_NAVIGATION=ON` | connect TargetLoc to `brat3_target` |
| `BRAT_USE_ROCK5_WHEELEDDRIVE=ON` | use the ROCK 5 wheel backend |
| `BRAT_ROCK5_MOTOR_DRY_RUN=ON` | run the sensor/planner chain without sending PWM |
| `BRAT_ROCK5_MOTOR_DRY_RUN=OFF` | enable real wheel output |

### Demo 1: TargetLoc viewer

Use this to inspect camera input, target detection and disparity:

```bash
rm -rf build-targetloc-rock5

cmake -S targetloc -B build-targetloc-rock5 \
  -DTARGETLOC_USE_ROCK5_V4L_CAMERA=ON

cmake --build build-targetloc-rock5 -j2

./build-targetloc-rock5/targetlocviewer/targetlocviewer
```

`Unable to get camera FPS` can be non-fatal on ROCK 5 if camera frames continue
to arrive.

### Demo 2: integrated dry-run

This uses the real cameras, LiDAR, TargetLoc and brat3 planner, but does not
send PWM commands to the wheels:

```bash
rm -rf build-demo-dry-run

cmake -S . -B build-demo-dry-run \
  -DBUILD_TESTING=OFF \
  -DBRAT_BUILD_TARGETLOC_NAVIGATION=ON \
  -DBRAT_USE_ROCK5_WHEELEDDRIVE=ON \
  -DBRAT_ROCK5_MOTOR_DRY_RUN=ON \
  -DTARGETLOC_USE_ROCK5_V4L_CAMERA=ON

cmake --build build-demo-dry-run \
  --target brat3_target \
  -j2

./build-demo-dry-run/brat3_target
```

### Demo 3: powered navigation

Clear the robot workspace and keep the hardware power cut-off accessible.

```bash
cmake -S . -B build-demo \
  -DBUILD_TESTING=OFF \
  -DBRAT_BUILD_TARGETLOC_NAVIGATION=ON \
  -DBRAT_USE_ROCK5_WHEELEDDRIVE=ON \
  -DBRAT_ROCK5_MOTOR_DRY_RUN=OFF \
  -DTARGETLOC_USE_ROCK5_V4L_CAMERA=ON

cmake --build build-demo \
  --target brat3_target \
  -j2

./build-demo/brat3_target
```

Typical successful startup includes:

```text
Waiting for a stable TargetLoc result
TargetLoc final coordinate: x=..., y=...
Locked target: x=..., y=...
Navigation approach target: x=..., y=..., standoff=0.14842 m
Rock 5 wheeledrive backend started.
Navigation ready. Press Enter to stop.
```

### Key configuration

The camera lateral cue is mapped to the expected LiDAR bearing by:

```cpp
targetAngleDeg = 56.0F * targetY - 1.0F;
```

The final TargetLoc association uses:

| Parameter | Value |
|---|---:|
| LiDAR range | 0.10–3.00 m |
| Bearing window | ±4° |
| Cluster gap | 0.12 m |
| Minimum cluster support | 2 points |
| Stable target lock | 3 samples within 0.03 m |

The selected LiDAR coordinate is calibrated using:

```text
x = 1.00255 * x_raw + 0.01556 m
y = 1.12269 * y_raw - 0.02852 m
```

The navigation goal uses a standoff of approximately `0.14842 m`. Long forward
motion is limited to the configured planning horizon (`simulationStep = 0.27 m`)
so the existing planner can re-evaluate the route between local plans.

Final heading recovery is performed only after the positional goal is complete.
It uses the net LEFT/RIGHT turn steps actually executed by the motor callback
instead of a fixed final 90-degree turn.

The existing `abandonPlan()` behaviour is unchanged. If the planner invalidates
the active transition, the current task is stopped and the diagnostic log can
show:

```text
[NAV SAFETY STOP] abandonPlan invalidated the current path: ...
```

Do not disable this behaviour to force the robot to continue through an
invalidated path.

### Hardware-specific motor parameters

Motor calibration depends on the individual robot. The parameters most likely
to require adjustment are in `wheeleddrive/Driving.h`:

```cpp
leftNeutralHighTimeNs
rightNeutralHighTimeNs
leftForwardSpeedScale
leftReverseSpeedScale
```

The validated ROCK 5 values are:

```text
left neutral        1,521,800 ns
right neutral       1,524,000 ns
left forward scale  0.990
left reverse scale  0.985
```

The motion model in `src/const.h` also depends on:

```cpp
MAX_SPEED
BETWEEN_WHEELS
WHEEL_SPEED_DEFAULT
WHEEL_SPEED_TURN
```

Current ROCK 5 values include:

```text
MAX_SPEED         0.1684 m/s
BETWEEN_WHEELS    0.1814 m
WHEEL_SPEED_TURN  0.281938
```

`BETWEEN_WHEELS` is an effective kinematic parameter used by the motion model,
not necessarily the physically measured wheel spacing.

### Main ROCK 5 integration files

| File | Purpose |
|---|---|
| `brat3_target.cpp` | TargetLoc-to-navigation orchestration |
| `targetloc/targetloc.cpp` | camera-guided LiDAR target estimation |
| `targetloc/targetlocreceiver.*` | stable target lock |
| `targetloc/rock5_V4Lcamera_backend.*` | ROCK 5 camera capture |
| `custom_robot.h` | LiDAR routing and motor callback |
| `src/configurator.cpp` | bounded forward planning |
| `src/task_controller.*` | final position and heading handling |
| `src/focused.cpp` | existing planner and `abandonPlan()` |
| `wheeleddrive/Driving.*` | ROCK 5 wheel output and hardware parameters |

### Troubleshooting

**Camera device not found:** inspect `/dev/video*` with `v4l2-ctl`; device
numbers can differ from the validated setup.

**Visual target detected but no TargetLoc coordinate:** the camera has detected
the target, but no valid LiDAR cluster currently satisfies the association
filters.

**`[NAV SAFETY STOP]`:** the existing planner invalidated the current
transition through `abandonPlan()`. Keep the safety behaviour enabled and check
the obstacle clearance / planner state instead.

### Limitations

* Navigation motion is open-loop; there are no wheel encoders.
* The target is acquired before navigation rather than continuously reacquired
  after every movement segment.
* TargetLoc localisation accuracy should not be interpreted as final robot
  endpoint accuracy.
* Stereo/disparity is diagnostic in the final system; calibrated LiDAR is the
  final metric TargetLoc source.
