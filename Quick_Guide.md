# Quick Guide

The following is a quick guide to using our programs (which will be updated as the programs change).

## Teleop:

### Joystick 1:

- Left Analog Stick Up/Down: Drive forwards and backwards
- Left Analog Stick Right/Left: Rotate in place
- Right Analog Stick Button: Reverse drive direction
- Right Analog Stick: Polar coordinate control of swivel
- Right Bumper: Turn the sweeper forwards (to collect debris) if polar swivel control is not being used. Extend linear lides otherwise.
- Right Trigger: Turn the sweeper backwards (to unjam the sweeper, when neccessary), if polar control is not being used. Retract linear slides otherwise.
- Left Bumper: Lower wall
- Left Trigger: Raise wall
- Y Button: Extend linear slides
- A Button: Retract linear slides
- Top Hat Left/Right: Swivel linear slides incrementally
- Top Hat Up/Down: Raise/Lower dumper

### Joystick 2:

- Left Bumper: Raise left climber triggerer
- Left Trigger: Lower left climber triggerer
- Right Bumper: Raise right climber triggerer
- Right Trigger: Lower right climber triggerer
- Right Analog Stick Up/Down: Extend/Retract linear slides
- Right Analog Stick Button: Move swivel to collection position
- Left Analog Stick Up/Down: Move robot lift motors (always at full power)
- A Button: Turn on reset automation, which sets swivel, dumper, and linear slides back to initial position. To end the automation, press A again, or operate sweeper.

## Autonomous:

There are several options to configure. The configuration of these options should show in telemetry.
Press and release the left bumper on gamepad 1 to decrement the selected setting, and the right bumper to increment the selected setting.
To move on to the next setting, press both bumpers and release them.

Current Options (option name before colon; choices listed after colon, with default option listed first):

- Color: RED, BLUE
- Start Wait Time: 0-30 seconds (0 is default)
- Starting Position: RAMP START, CORNER START, STRAIGHT START
- Path: PARK, COLLECTION, RAMP, OTHER_SIDE
- Beacon Scoring: ON, OFF
- Sweeper On: ON, OFF

## Drive Control:

Upon initialization, the telemetry display should cycle every few seconds (that time can be adjusted) through
forwards, backwards, left, and right. Run the program when the desired option is onscreen to start moving 
or rotating in that direction. Stop the program to stop moving or rotating. A few runs of this program should allow
for convenient robot return in autonomous testing and potentially other situations.