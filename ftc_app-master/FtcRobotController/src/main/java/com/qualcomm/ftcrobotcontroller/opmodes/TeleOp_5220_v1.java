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

    private double g1Stick1Xinit;
    private double g1Stick1Yinit;

    public void initialize ()
    {
        super.initialize();
        g1Stick1Xinit = gamepad1.left_stick_x;
        g1Stick1Yinit = gamepad1.left_stick_y;
    }

    public void loopBody()
    {
        double throttle  = (-(gamepad1.left_stick_y - g1Stick1Yinit));
        double direction =  (gamepad1.left_stick_x - g1Stick1Xinit);
        double right = throttle - direction;
        double left  = throttle + direction;

        // clip the right/left values so that the values never exceed +/- 1
        right = Range.clip(right, -2, 2);
        left  = Range.clip(left,  -2, 2);


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

        if (Math.abs (right) < JOYSTICK_THRESHOLD)
        {
            right = 0;
        }

        if (Math.abs(left) < JOYSTICK_THRESHOLD)
        {
            left = 0;
        }

        telemetry.addData("2", "Left: " + left);
        telemetry.addData("3", "Right: " + right);

        setLeftDrivePower(left);
        setRightDrivePower(right);

        if (gamepad1.left_stick_button)
        {
            g1Stick1Xinit = gamepad1.left_stick_x;
            g1Stick1Yinit = gamepad1.left_stick_y;
        }
    }

    public void main ()
    {
        while (opModeIsActive() && (gameTimer.time() /1000) < 120)
        {
            loopBody();
        }
    }
}
