# Robot -> fastDDS -> Qt

For transmitting data to a Qt interface which draws what the robot "sees". These data can be customised but in this setup they are the coordinates of the upper/lower bounds of robot, goal and initial disturbance (Di).

## Prerequisites

Fast CDR

`sudo apt-get install libfastcdr-dev`

Fast RTPS

`sudo apt-get install libfastrtps-dev`

Fast DDS tools

`sudo apt-get install fastddsgen fastdds-tools`

Qt packages

` sudo apt-get install qtdeclarative5-dev-tools qt5-qmake qt5-qmake-bin qtbase5-dev qtbase5-dev-tools libqwt-qt5-dev qt6-base-dev qt6-base-dev-tools qt6-tools-dev`

[CppTimer](https://github.com/berndporr/cppTimer) (only for running the demo)

## Generating fastDDS files

`fastddsgen ObjectPackage.idl`

## Build

`cmake .`
`make`

## Usage

* Basic demo: In one terminal window, run `./subscriber` and run `./publisher` in another.
* Qt demo: demonstrates paining the tracked contents of the CL tracker in a Qt window. Run `./test_tracker` in one terminal window and `./qt_window` in another.


## Credits

Based on Bernd Porr's [demo](https://github.com/berndporr/fastdds_demo).
