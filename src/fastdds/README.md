# FastDDS

For transmitting data to a Qt interface which draws what the robot "sees". These data can be customised but in this setup they are the coordinates of the upper/lower bounds of robot, goal and initial disturbance (Di).

## Generating fastDDS files

`fastddsgen Object.idl`

## Usage

In one terminal window, run `./subscriber`. If running the demo, run `./publisher` in another terminal window. Otherwise, run any demo program from `../../bratmobile` directory.


## Credits

Based on Bernd Porr's demo.
