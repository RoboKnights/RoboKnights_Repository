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

/*

TODO:

Add option to (try to) dump blocks in low goal before climbing ramp on our side
Add option to try to drive to other side of field (make sure its allowed), climb up the ramp of our color there, and position for scoring in the medium goal. probably terminate if gyro reads too much directional change.
Add ultrasonic sensor when we add rescue beacon detection?
*/

//Hello world.

//NOTE: Do NOT put waitFullCycle in loops. Only put in between other stuff

public class WiggleTest_5220 extends OpMode_5220
{
    public ProgramType getProgramType ()
    {
        return ProgramType.TELEOP;
    }

    public void initialize () //override
    {
        super.initialize(); //do everything in the original, common initialization.
        pinOn = false;
    }
    public void main ()
    {
        new DebuggerDisplayLoop().start();

        colorSensorDown.enableLed(true);
        waitFullCycle();
        colorSensorDown.enableLed(true);
        waitFullCycle();
        colorSensorFront.enableLed(false);
        waitFullCycle();
        colorSensorFront.enableLed(false);
        waitFullCycle();

        boolean prevLB2 = false;
        boolean prevRB2 = false;

        while (runConditions())
        {
            //LINEAR SLIDES:

            if (gamepad1.y || gamepad2.right_stick_y < -0.7) //up
            {
                slideMotor.setPower(1);
            }

            else if (gamepad1.a || gamepad2.right_stick_y > 0.7) //down
            {
                slideMotor.setPower(-1);
            }

            else
            {
                slideMotor.setPower(0);
            }

            //CLIMBER TRIGGERS:

            if (gamepad2.left_bumper && !prevLB2)
            {
                rightClimberServo.setPosition(rightClimberServo.getPosition() > 0.5 ? RIGHT_CLIMBER_INIT - CLIMBER_OFFSET : RIGHT_CLIMBER_INIT);
            }

            if (gamepad2.right_bumper && !prevRB2)
            {
                leftClimberServo.setPosition(leftClimberServo.getPosition() < 0.5 ? LEFT_CLIMBER_INIT + CLIMBER_OFFSET : LEFT_CLIMBER_INIT);
            }

            prevLB2 = gamepad2.left_bumper;
            prevRB2 = gamepad2.right_bumper;

            waitNextCycle();
        }
    }
}
