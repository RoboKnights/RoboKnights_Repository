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

import java.util.ArrayList;

public class Autonomous_5220_v1 extends OpMode_5220
{
    private static final boolean BLUE = true;
    private static final boolean RED = false;

    private boolean color = RED; //RED by default, of course it'll change when neccessary
    private int startWaitTime = 0; //in seconds, no need for non-integer numbers.
    private boolean smartDetectOn = false;

    public ProgramType getProgramType ()
    {
        return ProgramType.AUTONOMOUS;
    }

    private class ProgramKiller extends Thread
    {
        public void run()
        {
            while (opModeIsActive() && gameTimer.time() < 29.5)
            {

            }

            stopDrivetrain();
            //throw new RuntimeException("Program terminated by user.");
            //System.exit(0);
            //maybe add system.exit here and just remember to restart robot in between the match periods (assuming that's allowed)
        }
    }

    private class ConfigLoop extends Thread
    {
        private static final int UP = 1;
        private static final int DOWN = -1;

        private static final int COLOR = 0; //these are also their telemetry lines when added to 1.
        private static final int WAIT = 1;
        private static final int DETECT = 2;

        private static final int NUM_SETTINGS = 3;

        private int currentSetting = 0;

        private String[] telemetryLines = new String[NUM_SETTINGS];

        public void run ()
        {
            telemetry.addData("1", "AUTONOMOUS CONFIGURATION:");

            boolean prevL = false;
            boolean prevR = false;
            boolean bothPressed = false;

            while (phase < RUNNING) //ends when program is actually started. note to userL try to leave at least half a second in between config and running :D
            {
                //make sure this algorithm works properly.

                boolean l = touchSensorValue(0);
                boolean r = touchSensorValue(1);

                if (bothPressed)
                {
                    if (!l && !r)
                    {
                        nextSetting();
                        bothPressed = false;
                    }

                    continue;
                }

                if (l && r) //and of course, !bothPressed implicitly, since the program would not make it here if bothPressed were true.
                {
                    bothPressed = true;
                    prevL = false;
                    prevR = false;

                    continue;
                }

                if (l != prevL)
                {
                    if (!l) //released
                    {
                        adjustSetting(currentSetting, DOWN);
                    }

                    prevL = l;
                }

                if (r != prevR)
                {
                    if (!r) //released
                    {
                        adjustSetting(currentSetting, UP);
                    }

                    prevR = r;
                }

                //sleep(10); //not sure if neccessary
            }
        }

        private void nextSetting ()
        {
            if (telemetryLines[currentSetting].charAt(0) == '*') //change to string equals comparison if this doesn't work
            {
                telemetryLines[currentSetting] = telemetryLines[currentSetting].substring(1); //remove starter asterisk for old setting
            }

            currentSetting++;
            currentSetting = currentSetting % NUM_SETTINGS;

            telemetryLines[currentSetting] = "*" + telemetryLines[currentSetting]; //add starter asterisk to new setting

            writeLinesToTelemetry();
        }

        private void adjustSetting (int setting, int direction)
        {
            if (setting == COLOR)
            {
                color = !color;

                telemetryLines[COLOR] = ("Color: " + (color == RED ? "RED" : "BLUE"));
            }

            else if (setting == WAIT)
            {
                startWaitTime += direction;
                if (startWaitTime < 0)
                {
                    startWaitTime = 0;
                }

                telemetryLines[WAIT] = ("Wait Time: " + startWaitTime /*+ " seconds"*/);
            }

            else if (setting == DETECT)
            {
                smartDetectOn = !smartDetectOn;

                telemetryLines[DETECT] = ("Smart Detection: " + (smartDetectOn ? "ON" : "OFF"));
            }

            if (telemetryLines[currentSetting].charAt(0) != '*') //change to string equals comparison if this doesn't work
            {
                telemetryLines[currentSetting] = "*" + telemetryLines[currentSetting];
            }

            writeLinesToTelemetry();
        }

        private void writeLinesToTelemetry ()
        {
            for (int i = 0; i < telemetryLines.length; i++)
            {
                telemetry.addData("" + (i + 2), telemetryLines[i]);
            }
        }

        private void updateConfigDisplay() //identify current setting with asterisk before name of setting, or somewhere else.
        {
            String[] telemetryLines = new String [NUM_SETTINGS + 1]; //row zero is "Configuration", the rest are for settings. each setting's number is it's telemetry line.

        }
    }

    private Boolean getRescueBeaconColor () //experimental for the time being
    {
        double red = colorSensor.red();
        double blue = colorSensor.blue();
        double green = colorSensor.green();

        if (green > red / 2 && green > blue / 2) // indeterminate if there is too much green
        {
            return null;
        }

        double rbRatio = red / blue;
        double requiredRatio = 4.0;
        if (rbRatio > requiredRatio)
        {
            return RED;
        }

        else if (rbRatio < (1.0 / requiredRatio))
        {
            return BLUE;
        }

        else
        {
            return null;
        }
    }

    private boolean touchSensorValue (int port) //currently a placeholder
    {
        return false;
    }

    public void initialize () //override
    {
        super.initialize(); //do everything in the original, common initialization.
       // new ConfigLoop().start(); //uncomment once we figure out how to add the touch sensors for config
    }

    public void test()
    {
        resetDriveEncoders();
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
        //leftFrontMotor.setTargetPosition(5000);
        ///sleep(2000);
       // moveTime(500, 0.2);
        moveSimple (1000);
        sleep (1000);
        move (6);
        while (opModeIsActive())
        {

        }

        stopDrivetrain();

       // System.exit(0);
    }


    public void autonomous ()
    {
        move (24); //12 = one foot, 24 = one floor tile
        if (BLUE)
        {
            rotate(45);
        }

        else
        {
            rotate(-45);
        }
        move(67.88);
        if (BLUE)
        {
            rotate(-45);
        }

        else
        {
            rotate(45);
        }

        //detect first side
        move(12);
        //detect second side
        //make decision on which side it is.
        //press appropriate button

        if (BLUE)
        {
            rotate(90);
        }

        else
        {
            rotate(-90);
        }

        //go forward, dump climbers and move back

        if (BLUE)
        {
            rotate(-45);
        }

        else
        {
            rotate(45);
        }

        move (-33.94);

        if (BLUE)
        {
            rotate(90);
        }

        else
        {
            rotate(-90);
        }

        moveTime(30000, 99); //drive at full power up the mountain until ProgramKiller kills the program at the 30 second mark.

    }

    public void main ()
    {
        new ProgramKiller().start();
        new DebuggerDisplayLoop().start();
       // telemetry.clearData();
        telemetry.addData ("2", "hello world!");
        test();
        //autonomous();
    }


}
