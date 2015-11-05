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

public class TeleOp_5220_v1 extends OpMode_5220 //this is a comment. It is a long comment.
{
    private static final double JOYSTICK_THRESHOLD = 0.08; //below this joysticks won't cause movement.
    private static final double SLOW_POWER = 0.15;

    private static final double SWIVEL_INCREMENT = 0.01;
    private static final double SWIVEL_INCREMENT_TIME = 70; //in millis, every incrmeent time, it goes 0.01 counts. about 24 increments to go 180 then.
    private static final double SWIVEL_360 = 0.242;

    private static final double ARM_INCREMENT = 0.04;
    private static final double ARM_INCREMENT_TIME = 30; //in millis, every incrmeent time, it goes 0.01 counts. about 24 increments to go 180 then.

    private static final double HOOK_TILT_INCREMENT = 0.025;

    private double g1Stick1Xinit;
    private double g1Stick1Yinit;


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

    public void initialize ()
    {
        super.initialize();
        swivelServo.setPosition(1); //full range is 6.25 rotation, approximately. 1 is collection position
        armServo.setPosition(0.14);
        hookTiltServo.setPosition(1);
        g1Stick1Xinit = gamepad1.left_stick_x;
        g1Stick1Yinit = gamepad1.left_stick_y;
    }

    public void loop5220()
    {
        Stopwatch topHatXTime = null;
        Stopwatch topHatYTime = null;
        Stopwatch hookTiltTime = null;

        boolean prevTopHatUp1 = false; //maybe change these initialization if they mess something up
        boolean prevTopHatDown1 = false;
        boolean prevTopHatLeft1 = false;
        boolean prevTopHatRight1 = false;
        boolean prevLB = false;
        boolean prevLT = false;

        while (runConditions())
        {
            //DRIVETRAIN CONTROL:

            double throttle = (-(gamepad1.left_stick_y - g1Stick1Yinit));
            double direction = (gamepad1.left_stick_x - g1Stick1Xinit);
            double right = throttle - direction;
            double left = throttle + direction;

            // clip the right/left values so that the values never exceed +/- 1
            right = Range.clip(right, -2, 2);
            left = Range.clip(left, -2, 2);

            if (gamepad1.left_stick_button)
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

            setLeftDrivePower(left);
            setRightDrivePower(right);

            if (gamepad1.left_stick_button)
            {
                g1Stick1Xinit = gamepad1.left_stick_x;
                g1Stick1Yinit = gamepad1.left_stick_y;
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

            //DOOR CONTROL:

            moveDoor(gamepad1.b ? OPEN : CLOSE);

            //SWIVEL CONTROL:

            if ((gamepad1.dpad_left) && (!prevTopHatLeft1 || (topHatXTime != null && topHatXTime.time() > SWIVEL_INCREMENT_TIME)))
            {
                double newPosition = swivelServo.getPosition() + SWIVEL_INCREMENT;
                if (newPosition > 1) newPosition = 1;
                swivelServo.setPosition(newPosition);
                topHatXTime = new Stopwatch();
            }

            if ((gamepad1.dpad_right) && (!prevTopHatRight1 || (topHatXTime != null && topHatXTime.time() > SWIVEL_INCREMENT_TIME)))
            {
                double newPosition = swivelServo.getPosition() - SWIVEL_INCREMENT;
                if (newPosition < 0) newPosition = 0;
                swivelServo.setPosition(newPosition);
                topHatXTime = new Stopwatch();
            }

            if (!gamepad1.dpad_left && !gamepad1.dpad_right)
            {
                topHatXTime = null;
            }

            //ARM CONTROL:

            if ((gamepad1.dpad_up) && (!prevTopHatUp1 || (topHatYTime != null && topHatYTime.time() > ARM_INCREMENT_TIME)))
            {
                double newPosition = armServo.getPosition() + ARM_INCREMENT;
                if (newPosition > 1) newPosition = 1;
                armServo.setPosition(newPosition);
                topHatYTime = new Stopwatch();
            }

            if ((gamepad1.dpad_down) && (!prevTopHatDown1 || (topHatYTime != null && topHatYTime.time() > ARM_INCREMENT_TIME)))
            {
                double newPosition = armServo.getPosition() - ARM_INCREMENT;
                if (newPosition < 0) newPosition = 0;
                armServo.setPosition(newPosition);
                topHatYTime = new Stopwatch();
            }

            if (!gamepad1.dpad_down && !gamepad1.dpad_up)
            {
                topHatYTime = null;
            }

            //HOOK TILT CONTROL:

            if (gamepad1.left_bumper != prevLB)
            {
                if (gamepad1.left_bumper)
                {
                    double newPosition = hookTiltServo.getPosition() + HOOK_TILT_INCREMENT;
                    hookTiltServo.setPosition(Math.min(newPosition, 1.0));
                }
            }

            else if ((gamepad1.left_trigger > 0.7) != prevLT)
            {
                if (gamepad1.left_trigger > 0.7)
                {
                    double newPosition = hookTiltServo.getPosition() - HOOK_TILT_INCREMENT;
                    hookTiltServo.setPosition(Math.max(newPosition, 0.0));
                }
            }

            //HOOK EXTENSION CONTROL:

            if (gamepad1.y) //up
            {
                hookMotor.setPower(1);
            }

            else if (gamepad1.a)
            {
                hookMotor.setPower(-1);
            }

            else
            {
                hookMotor.setPower(0);
            }

            //Previous value settings:

            prevTopHatUp1 = gamepad1.dpad_up;
            prevTopHatDown1 = gamepad1.dpad_down;
            prevTopHatRight1 = gamepad1.dpad_right;
            prevTopHatLeft1 = gamepad1.dpad_left;
            prevLB = gamepad1.left_bumper;
            prevLT = gamepad1.left_trigger > 0.7;
        }
    }

    public void main ()
    {
        new DebuggerDisplayLoop().start();
        for (DcMotor dcm: driveMotors) dcm.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        loop5220();
    }
}