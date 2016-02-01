/* Copyright (c) 2015 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.*;

//MAKE THING SO THAT MOVING THE ANALOG STICK TO A ROTATIONAL POSITION (and pressing a button) WILL ROTATIONALLY INSTANTLY MOVE THE ARM TO THAT LOCATION. USE TRIG TO CONVERT FROM X Y TO R THETA.

// change dumper up/down to top hat up/down (instead of x and b)
//change the wall control to be both alvin and eric, same buttons
//add slow drive control

public class TeleOp_5220_v1 extends OpMode_5220 //this is a comment. It is a long comment.
{
    private static final double JOYSTICK_THRESHOLD = 0.08; //below this joysticks won't cause movement.
    private static final double SLOW_POWER = 0.10;


    private static final double SWIVEL_INCREMENT = 0.00658; //changed from 0.005

    private static final double SWIVEL_INCREMENT_TIME = 60; //in millis, every incrmeent time, it goes 0.01 counts. about 24 increments to go 180 then.
    private static final double SWIVEL_INERTIA_CORRECTION_MULTIPLIER = 0.5;

    private static final double DUMPER_INCREMENT = 0.025; //changed from 0.005
    private static final double DUMPER_INCREMENT_TIME = 60; //in millis, every incrmeent time, it goes 0.01 counts. about 24 increments to go 180 then.


    private static final double ARM_INCREMENT = 0.04;
    private static final double ARM_INCREMENT_TIME = 30; //in millis, every incrmeent time, it goes 0.01 counts. about 24 increments to go 180 then.


    private static final double HOOK_TILT_INCREMENT = 0.072;

    private static final double POLAR_CONTROL_R_THRESHOLD = 0.8; //max R value should be about 1

    private double g1Stick1Xinit;
    private double g1Stick1Yinit;

    private boolean reverseDriveOn = false;
    private boolean slowDriveOn = false;


    /*
    private void updateTopHat()
    {
        if (gamepad1.)
    }

    private void updatePrevTopHat ()
    {

    }
*/

    public ProgramType getProgramType ()
    {
        return ProgramType.TELEOP;
    }

    public double[] cartesianToPolar (double x, double y) //returns array with polar coordinates, theta between 0 and 2 pi.
    {
        double r = Math.sqrt((x * x) + (y * y));
        double theta = Math.atan2(x, y);
        /*
        if (x < 0) //make sure this works
        {
            theta = theta + Math.PI;
        }
        */
        theta = (theta + (2 * Math.PI)) % (2 * Math.PI);
        //record values here
        /*
        if (theta < 0)
        {
            theta = theta + (2 * Math.PI);
        }
        */
        double[] toReturn = new double[2];

        toReturn[0] = r;
        toReturn[1] = theta;
        return toReturn;
    }

    public boolean cartesianToPolar (double[] d) //changes two value array from x and y to r and theta, return true if it succeeds in doing so
    {
        if (d.length != 2) return false;
        double[] polarArray = cartesianToPolar(d[0], d[1]);
        d[0] = polarArray[0];
        d[1] = polarArray[1];
        return true;
    }

    public double radiansToDegrees (double radians)
    {
        double circleFraction = radians / (2.0 * Math.PI);
        double degrees = circleFraction * 360.0;
        return degrees;
    }

    public double degreesToSwivelPosition (double degrees) //degrees on analog stick are measured in traditional way, with zero starting in the first quadrant
    {
        double swivelPositionAtZeroDegrees = SWIVEL_INIT - (0.75 * SWIVEL_360);
        double swivelPosition = swivelPositionAtZeroDegrees + ((degrees / 360.0) * SWIVEL_360);
        swivelPosition = swivelPosition - (SWIVEL_360 / 4);
        swivelPosition = swivelPosition + ((SWIVEL_360 / 360.0) * 13.5); //brute force fix, eliminate if possible later.
        return swivelPosition;
    }

    public void initialize ()
    {
        super.initialize();
        g1Stick1Xinit = gamepad1.left_stick_x;
        g1Stick1Yinit = gamepad1.left_stick_y;
    }

    public void loop5220()
    {
        Stopwatch topHatXTime = null;
        Stopwatch topHatYTime = null;
        Stopwatch dumperTime = null;
        Stopwatch hookTiltTime = null;

        double swivelMovementStart = 0.0;

        boolean prevTopHatUp1 = false; //maybe change these initialization if they mess something up
        boolean prevTopHatDown1 = false;
        boolean prevTopHatLeft1 = false;
        boolean prevTopHatRight1 = false;
        boolean prevTopHatUp2 = false; //maybe change these initialization if they mess something up
        boolean prevTopHatDown2 = false;
        boolean prevTopHatLeft2 = false;
        boolean prevTopHatRight2 = false;
        boolean prevX1 = false;
        boolean prevRSB1 = false;
        boolean prevB1 = false;
        boolean prevLB = false;
        boolean prevLT = false;
        boolean prevBack = false;
        boolean prevY2 = false;
        boolean prevLB2 = false;
        boolean prevLT2 = false;

        while (runConditions())
        {
            //DRIVETRAIN CONTROL:

            double throttle = (-(gamepad1.left_stick_y - g1Stick1Yinit));
            double direction = (gamepad1.left_stick_x - g1Stick1Xinit);

            if (reverseDriveOn)
            {
                throttle = -throttle;
            }

            double right = throttle - direction;
            double left = throttle + direction;

            // clip the right/left values so that the values never exceed +/- 1
            right = Range.clip(right, -2, 2);
            left = Range.clip(left, -2, 2);

            if (false) //change to button later.
            {
                if (right > SLOW_POWER)
                {
                    right = SLOW_POWER;
                }

                if (right < -SLOW_POWER)
                {
                    right = -SLOW_POWER;
                }

                if (left > SLOW_POWER)
                {
                    left = SLOW_POWER;
                }

                if (left < -SLOW_POWER)
                {
                    left = -SLOW_POWER;
                }
            }

            else
            {

                if (right > 1)
                {
                    right = 1;
                }

                if (right < -1)
                {
                    right = -1;
                }

                if (left > 1)
                {
                    left = 1;
                }

                if (left < -1)
                {
                    left = -1;
                }
            }

            if (Math.abs(right) < JOYSTICK_THRESHOLD)
            {
                right = 0;
            }

            if (Math.abs(left) < JOYSTICK_THRESHOLD)
            {
                left = 0;
            }

            if (left == 0 && right == 0)
            {
                if (gamepad1.right_stick_y < -0.4)
                {
                    left = -SLOW_POWER;
                    right = -SLOW_POWER;
                }

                else if (gamepad1.right_stick_y > 0.4)
                {
                    left = SLOW_POWER;
                    right = SLOW_POWER;
                }

            }

            setLeftDrivePower(left);
            setRightDrivePower(right);

            if (gamepad1.start)
            {
                g1Stick1Xinit = gamepad1.left_stick_x;
                g1Stick1Yinit = gamepad1.left_stick_y;
            }
/*
            if (gamepad1.back != prevBack && !gamepad1.back) //acts on button release
            {
                reverseDriveOn = !reverseDriveOn;
            }
*/

            if (gamepad1.right_stick_button != prevRSB1 && gamepad1.right_stick_button) //acts on button press
            {
                reverseDriveOn = !reverseDriveOn;
            }

            //SWEEPER CONTROL:

            double sweeperPower = 0;

            if (gamepad1.right_bumper)
            {
                sweeperPower = 1;
            }

            else if (gamepad1.right_trigger > 0.7) //not entirely sure we can or will ever need to do this, move the sweeper in reverse.
            {
                sweeperPower = -1;
            }

            setMotorPower(sweeperMotor, sweeperPower);

            //DUMPER CONTROL:

            //moveDumper(gamepad1.b ? UP : DOWN);

            if (gamepad1.left_trigger > 0.7 || gamepad2.left_bumper) //color sensor added for testing
            {
                moveWall(UP);
                //colorSensorFront.enableLed(false);
                //colorSensorFront.enableLed(false);

            }
            else if (gamepad1.left_bumper || gamepad2.left_trigger > 0.7) {
                moveWall(DOWN);
                //colorSensorFront.enableLed(true);
                //colorSensorFront.enableLed(true);
            }

            //MOUNTAIN HOOK CONTROL





            //ADD SOME WAY TO DISABLE P1 ARM CONTROL WHILE P2 IS RUNNING AN ARM MOTION SUBROUTINE.
            //SWIVEL CONTROL:



            if ((gamepad1.dpad_right) && (!prevTopHatRight1 || (topHatXTime != null && topHatXTime.time() > SWIVEL_INCREMENT_TIME)))
            {
                double newPosition = swivelServo.getPosition() + SWIVEL_INCREMENT;
                if (newPosition > 1) newPosition = 1;
                swivelServo.setPosition(newPosition);
                topHatXTime = new Stopwatch();
            }

            if ((gamepad1.dpad_left) && (!prevTopHatLeft1 || (topHatXTime != null && topHatXTime.time() > SWIVEL_INCREMENT_TIME)))
            {
                double newPosition = swivelServo.getPosition() - SWIVEL_INCREMENT;
                if (newPosition < 0) newPosition = 0;
                swivelServo.setPosition(newPosition);
                topHatXTime = new Stopwatch();
            }

            if (!gamepad1.dpad_left && !gamepad1.dpad_right)
            {
                topHatXTime = null;

                if (gamepad2.a)
                {
                    moveSwivel(RED_MEDIUM);
                }

                else if (gamepad2.b)
                {
                    moveSwivel(BLUE_MEDIUM);
                }

                else if (gamepad2.x)
                {
                    moveSwivel(RED_HIGH);
                }

                else if (gamepad2.y)
                {
                    moveSwivel(BLUE_HIGH);
                }

                else if (gamepad2.right_stick_button)
                {
                    moveSwivel(COLLECT);
                }
            }

            //DUMPER CONTROL:

            if ((gamepad1.dpad_up) && (!prevTopHatUp1 || (dumperTime != null && dumperTime.time() > DUMPER_INCREMENT_TIME)))
            {
                double newPosition = leftDumpServo.getPosition() + DUMPER_INCREMENT;
                if (newPosition > 1) newPosition = 1;
                moveDumper(newPosition);

                dumperTime = new Stopwatch();
            }

            if ((gamepad1.dpad_down) && (!prevTopHatDown1 || (dumperTime != null && dumperTime.time() > DUMPER_INCREMENT_TIME)))
            {
                double newPosition = leftDumpServo.getPosition() - DUMPER_INCREMENT;
                if (newPosition < DOWN - 0.02) newPosition = DOWN;
                moveDumper(newPosition);

                dumperTime = new Stopwatch();
            }

            if (!gamepad1.dpad_down && !gamepad1.dpad_up)
            {
                dumperTime = null;

                if (gamepad1.b || gamepad2.dpad_down)
                {
                    moveDumper(DOWN);
                }

                else if (gamepad1.x || gamepad2.dpad_up)
                {
                    moveDumper(DOWN + 0.184); //fine-tune value soon
                }
            }


            //HOOK TILT CONTROL:

            //HOOK EXTENSION CONTROL:

            if (gamepad1.y || gamepad2.right_bumper) //up
            {
                slideMotor.setPower(1);
            }

            else if (gamepad1.a || gamepad2.right_trigger > 0.7)
            {
                slideMotor.setPower(-1);
            }

            else
            {
                slideMotor.setPower(0);
            }

            //LIFT MOTOR CONTROL:
/*
            if (gamepad2.right_stick_y > 0.7)
            {
                setLiftPower(1);
            }

            else if (gamepad2.right_stick_y < -0.7)
            {
                setLiftPower(-1);
            }

            else
            {
                setLiftPower(0);
            }
*/
            //CLIMBER TRIGGERER CONTROL
/*
            if (gamepad2.y != prevY2) //tServo 1 is port 3, the one on the left, looking at the robot with the sweeper at the front and hook extension on the back.
            {
                if (gamepad2.y)
                {
                    triggerServo1.setPosition(triggerServo1.getPosition() == 1 ? 0.232 : 1);
                    triggerServo2.setPosition(triggerServo2.getPosition() == 0 ? 0.75 : 0);
                }
            }
*//*
            if (gamepad2.left_bumper != prevLB2) //tServo 1 is port 3, the one on the left, looking at the robot with the sweeper at the front and hook extension on the back.
            {
                if (gamepad2.left_bumper)
                {
                    triggerServo2.setPosition(triggerServo2.getPosition() == 0 ? 0.75 : 0);
                }
            }

            if (gamepad2.left_trigger > 0.7 != prevLT2) //tServo 1 is port 3, the one on the left, looking at the robot with the sweeper at the front and hook extension on the back.
            {
                if (gamepad2.left_trigger > 0.7)
                {
                    triggerServo1.setPosition(triggerServo1.getPosition() == 1 ? 0.232 : 1);
                }
            }
*/
            //climber fling control
/*
            double flingValue = gamepad1.right_stick_y;
            flingValue = Math.abs(flingValue);
            flingValue = 1.0 - flingValue;
            flingValue = Math.min(1.0, flingValue);
            flingValue = Math.max(0.0, flingValue);

            climberServo.setPosition(flingValue);
*/
            //Previous value settings:

            prevTopHatUp1 = gamepad1.dpad_up;
            prevTopHatDown1 = gamepad1.dpad_down;
            prevTopHatRight1 = gamepad1.dpad_right;
            prevTopHatLeft1 = gamepad1.dpad_left;
            prevTopHatUp2 = gamepad2.dpad_up;
            prevTopHatDown2 = gamepad2.dpad_down;
            prevTopHatRight2 = gamepad2.dpad_right;
            prevTopHatLeft2 = gamepad2.dpad_left;
            prevRSB1 = gamepad1.right_stick_button;
            prevX1 = gamepad1.x;
            prevB1 = gamepad1.b;
            prevLB = gamepad1.left_bumper;
            prevLT = gamepad1.left_trigger > 0.7;
            prevBack = gamepad1.back;
            prevY2 = gamepad2.y;
            prevLB2 = gamepad2.left_bumper;
            prevLT2 = gamepad2.left_trigger > 0.7;
        }
    }

    public void main ()
    {
        new DebuggerDisplayLoop().start();
        for (DcMotor dcm: driveMotors) dcm.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        colorSensorFront.enableLed(false);
        loop5220();
    }
}