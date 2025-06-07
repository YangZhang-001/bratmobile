# FastDDS

For transmitting data to a Qt interface which draws what the robot "sees". These data can be customised but in this setup they are the coordinates of the upper/lower bounds of robot, goal and initial disturbance (Di).

## Prerequisites

Fast CDR

`sudo apt-get install libfastcdr-dev`

Fast RTPS

`sudo apt-get install libfastrtps-dev`

Fast DDS tools

`sudo apt-get install fastddsgen fastdds-tools`

[CppTimer](https://github.com/berndporr/cppTimer) (only for running the demo)

## Generating fastDDS files

`fastddsgen ObjectPackage.idl`

## Build

`cmake .`
`make`

## Usage

In one terminal window, run `./subscriber`. If running the demo, run `./publisher` in another terminal window. Otherwise, run any demo program from `../../bratmobile` directory.


## Credits

Based on Bernd Porr's [demo](https://github.com/berndporr/fastdds_demo).
