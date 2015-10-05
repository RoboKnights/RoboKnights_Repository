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

public class Autonomous_5220_v1 extends OpMode_5220
{
    private class ProgramKiller extends Thread
    {
        public void run()
        {
            while (opModeIsActive())
            {

            }

            //throw new RuntimeException("Program terminated by user.");
            System.exit(0);
        }
    }

    public void test()
    {
        /*
        telemetry.addData ("1", "init");
        sleep (1000);
        //servoL.setPosition(0.1);
        telemetry.addData ("2", "servo moving");
        sleep(2000);
        leftFrontMotor.setPower(0.5);
        telemetry.addData ("3", "dc motor moving");
        sleep (3000);
        leftFrontMotor.setPower(0);
        telemetry.addData("4", "dc motor stopped");
        sleep (1000);
        // servoL.setPosition(0.5);
        telemetry.addData("5", "all stoppedd");
        */

        //moveTime(2000);
        setDrivePower(0.99);

        while (opModeIsActive())
        {

        }

        stopDrivetrain();
    }

    public void main ()
    {
        //new ProgramKiller().start();
        telemetry.addData ("2", "hello world!");
        test();
    }


}
