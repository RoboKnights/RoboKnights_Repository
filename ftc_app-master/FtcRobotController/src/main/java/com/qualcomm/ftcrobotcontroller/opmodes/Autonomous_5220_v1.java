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

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.*;

import java.util.ArrayList;

/*

TODO:

Add option to (try to) dump blocks in low goal before climbing ramp on our side
Add option to try to drive to other side of field (make sure its allowed), climb up the ramp of our color there, and position for scoring in the medium goal. probably terminate if gyro reads too much directional change.
Add ultrasonic sensor when we add rescue beacon detection?
*/


public class Autonomous_5220_v1 extends OpMode_5220
{
    public static final int LOW_GOAL_AND_COLLECT_ON_SAME_SIDE = 0;
    public static final int LOW_GOAL_AND_COLLECT_ON_OTHER_SIDE = 1;
    public static final int RAMP = 2;
    public static final int DEFENSE = 3;
    public static final int NUM_PATHS = 4;


    private boolean color = BLUE; //RED by default, of course it'll change when neccessary
    private int path = 0;
    private int startWaitTime = 0; //in seconds, no need for non-integer numbers.
    private boolean sweeperOn = true;

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
            writeToLog("Program Killer has terminated the program.");
        }
    }

    private class ConfigLoop extends Thread //uses touch sensors to configure robot
    {
        private static final int UP = 1;
        private static final int DOWN = -1;

        private static final int COLOR = 0; //these are also their telemetry lines when added to 1.
        private static final int WAIT = 1;
        private static final int SWEEP = 2;
        private static final int PATH = 3;

        private static final int NUM_SETTINGS = 4;

        private int currentSetting = 0;

        private String[] telemetryLines = new String[NUM_SETTINGS];


        public void run ()
        {
            for (int i = 0; i < telemetryLines.length; i++) telemetryLines[i] = "";
            telemetry.addData("1", "AUTONOMOUS CONFIGURATION:");
            telemetryLines[COLOR] = ("Color: " + (color == RED ? "RED" : "BLUE")); //maybe add starter asterisk here. Not sure if it is neccessary.
            telemetryLines[WAIT] = ("Wait Time (in seconds): " + startWaitTime /*+ " seconds"*/);
            telemetryLines[SWEEP] = ("Sweeper: " + (sweeperOn ? "ON" : "OFF"));
            telemetryLines[PATH] = ("Path: " + pathToString(path));
            writeLinesToTelemetry();

            boolean prevL = false;
            boolean prevR = false;
            boolean bothPressed = false;

            while (phase < RUNNING) //ends when program is actually started. note to userL try to leave at least half a second in between config and running :D
            {
                //make sure this algorithm works properly.
                boolean l;
                boolean r;

                l = touchSensor1.isPressed();
                r = touchSensor2.isPressed();

                if (!l && !r)
                {
                    l = gamepad1.left_bumper;
                    r = gamepad1.right_bumper;
                }


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

                telemetryLines[WAIT] = ("Wait Time(in seconds): " + startWaitTime /*+ " seconds"*/);
            }

            else if (setting == SWEEP)
            {
                sweeperOn = !sweeperOn;

                telemetryLines[SWEEP] = ("Sweeper: " + (sweeperOn ? "ON" : "OFF"));
            }

            else if (setting == PATH) //FINISH
            {
                path = (path + direction) % NUM_PATHS;
                telemetryLines[PATH] = ("Path: " + pathToString(path));
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

        private String pathToString (int path)
        {
            switch (path)
            {
                case LOW_GOAL_AND_COLLECT_ON_SAME_SIDE: return "LOW_GOAL_AND_COLLECT_ON_SAME_SIDE";
                case LOW_GOAL_AND_COLLECT_ON_OTHER_SIDE: return "LOW_GOAL_AND_COLLECT_ON_OTHER_SIDE";
                case RAMP: return "RAMP";
                case DEFENSE: return "DEFENSE";
                default: return "Error: Invalid Path Number.";
            }
        }
    }



    private boolean touchSensorValue (int port) //currently a placeholder
    {
        return false;
    }

    public void initialize () //override
    {
        super.initialize(); //do everything in the original, common initialization.
        new ConfigLoop().start(); //uncomment once we figure out how to add the touch sensors for config
    }

    public void test()
    {

    }


    public void autonomous ()
    {/*
        if (color == RED)
        {
            move (24); //12 = one foot, 24 = one floor tile
           // rotate(-45);
            rotateEncoder(-6);
            move(67.88);
            //rotate(45);
            rotateEncoder(-6);
            //detect first side
            move(12);
            //detect second side, make decision where to push
            //rotate(-90);
            rotateEncoder(-12);
            //go forward, dump climbers, and come back.
            //rotate(45);
            rotateEncoder(6);
            move (-33.94);
            rotateEncoder(12);
            moveTime(30000, 0.99); //drive at full power up the mountain until ProgramKiller kills the program at the 30 second mark.
        }

        else if (color == BLUE)
        {
            move (24); //12 = one foot, 24 = one floor tile
            // rotate(-45);
            rotateEncoder(6);
            move(48); //was 67.88
            //rotate(45);
            rotateEncoder(6);
            //detect first side
            move(12);
            //detect second side, make decision where to push
            //rotate(-90);
            rotateEncoder(12);
            //go forward, dump climbers, and come back.
            //rotate(45);
            rotateEncoder(-6);
            move (-33.94);
            rotateEncoder(-12);
            moveTime(30000, 0.99); //drive at full power up the mountain until ProgramKiller kills the program at the 30 second mark.
        }
        */

        if (color == BLUE)
        {
            if (sweeperOn) setMotorPower(sweeperMotor, 1);
            move(70);
            if (sweeperOn) setMotorPower(sweeperMotor, 0);
            sleep(250);
            hookTiltServo.setPosition(1);
            sleep(600);
            rotateEncoder(5.32); //was 3.62
            sleep(350);
            move(4.35);
            sleep(350);
            flingClimbers();
            sleep(350);
            move(-3);
            sleep(350);
            rotateEncoder(-5.5);
            sleep(300);

            if (path == LOW_GOAL_AND_COLLECT_ON_SAME_SIDE)
            {
                if (sweeperOn) setMotorPower(sweeperMotor, -1);
                move(-31.5); //was 41.5
                if (sweeperOn) setMotorPower(sweeperMotor, 0);
                sleep(400);
                rotateEncoder(-13.0);
                /*
                armServo.setPosition(1);
                sleep(2500);
                swivelServo.setPosition(SWIVEL_INIT - SWIVEL_180);
                sleep(2500);
                move(-8);//was 51.6
                */
                /*
                sleep(1000);
                armServo.setPosition(0.68);
                sleep(1000);
                moveDoor(OPEN);
                sleep(3000);
                moveDoor(CLOSE);
                armServo.setPosition(1);
                sleep(700);
                move(15);
                swivelServo.setPosition(SWIVEL_INIT);
                setDrivePower(0.7);
                sleep(2000);
                armServo.setPosition(0.14);
                setDrivePower(0);
                sleep(1000);
                */
            }

            else if (path == RAMP)
            {
                if (sweeperOn) setMotorPower(sweeperMotor, -1);
                move(-21.5); //was 41.5
                if (sweeperOn) setMotorPower(sweeperMotor, 0);
                sleep(700);
                rotateEncoder(-13.0);
                armServo.setPosition(1);
                sleep(1500);
                swivelServo.setPosition(SWIVEL_INIT - SWIVEL_180);
                sleep(1200);
                move(-51.6);//was 51.6
            }
        }

        else if (color == RED)
        {
            if (sweeperOn) setMotorPower(sweeperMotor, 1);
            move(64);
            if (sweeperOn) setMotorPower(sweeperMotor, 0);
            sleep(250);
            hookTiltServo.setPosition(1);
            sleep(600);
            rotateEncoder(-3.79); //was 4.32
            sleep(350);
            move(1.2);
            sleep(350);
            flingClimbers();
            sleep(350);
            move(-3);
            sleep(450);
            rotateEncoder(3.5);
            sleep(400);

            if (path == LOW_GOAL_AND_COLLECT_ON_SAME_SIDE)
            {
                if (sweeperOn) setMotorPower(sweeperMotor, -1);
                move(-31.5); //was 41.5
                if (sweeperOn) setMotorPower(sweeperMotor, 0);
                sleep(400);
                rotateEncoder(13.0);
                /*
                armServo.setPosition(1);
                sleep(2500);
                swivelServo.setPosition(SWIVEL_INIT - SWIVEL_180);
                sleep(2500);
                move(-8);//was 51.6
                */
                /*
                sleep(1000);
                armServo.setPosition(0.68);
                sleep(1000);
                moveDoor(OPEN);
                sleep(3000);
                moveDoor(CLOSE);
                armServo.setPosition(1);
                sleep(700);
                move(15);
                swivelServo.setPosition(SWIVEL_INIT);
                setDrivePower(0.7);
                sleep(2000);
                armServo.setPosition(0.14);
                setDrivePower(0);
                sleep(1000);
                */
            }

            else if (path == RAMP)
            {
                if (sweeperOn) setMotorPower(sweeperMotor, -1);
                move(-21.5); //was 41.5
                if (sweeperOn) setMotorPower(sweeperMotor, 0);
                sleep(700);
                rotateEncoder(-13.0);
                armServo.setPosition(1);
                sleep(1500);
                swivelServo.setPosition(SWIVEL_INIT - SWIVEL_180);
                sleep(1200);
                move(-51.6);//was 51.6
            }

        }
    }

    public void main ()
    {
        //new ProgramKiller().start(); //PROGRAM KILLER SCREWS UP AUTONOMOUS.
        new DebuggerDisplayLoop().start();
        sleep(startWaitTime * 1000);
       // test();
        autonomous();
    }


}
