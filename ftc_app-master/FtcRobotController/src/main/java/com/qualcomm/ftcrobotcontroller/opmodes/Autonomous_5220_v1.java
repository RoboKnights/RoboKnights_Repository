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

//Hello world.

//NOTE: Do NOT put waitFullCycle in loops. Only put in between other stuff

public class Autonomous_5220_v1 extends OpMode_5220
{
    public static final int LOW_GOAL_AND_COLLECT_ON_SAME_SIDE = 0;
    public static final int LOW_GOAL_AND_COLLECT_ON_OTHER_SIDE = 1;
    public static final int RAMP = 2;
    public static final int DEFENSE = 3;
    public static final int NUM_PATHS = 4;

    public static final int START_RAMP = 0;
    public static final int START_CORNER = 1;
    public static final int START_STRAIGHT = 2;
    public static final int NUM_STARTS = 3;

    public static final double lineBlockedTime = 17000;

    private boolean color = BLUE; //RED by default, of course it'll change when neccessary
    private int startPosition = START_RAMP;
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

    private class ConfigLoop extends Thread //uses gamepad 1 left and right bumpers to configure. Will probably change to using analog stick or top hat with up and down for changing options.
    {
        private static final int UP = 1;
        private static final int DOWN = -1;

        private static final int COLOR = 0; //these are also their telemetry lines when added to 1.
        private static final int START = 1;
        private static final int WAIT = 2;
        private static final int SWEEP = 3;
        private static final int PATH = 4;

        private static final int NUM_SETTINGS = 5; //always make sure this is correct.

        private int currentSetting = 0;

        private String[] telemetryLines = new String[NUM_SETTINGS];


        public void run ()
        {
            for (int i = 0; i < telemetryLines.length; i++) telemetryLines[i] = "";
            telemetryLines[COLOR] = ("Color: " + (color == RED ? "RED" : "BLUE")); //maybe add starter asterisk here. Not sure if it is neccessary.
            telemetryLines[START] = ("Start Position " + (startPosition == START_RAMP ? "RAMP" : "CORNER"));
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

                l = gamepad1.left_bumper;
                r = gamepad1.right_bumper;


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

                if (gamepad1.y)
                {
                    colorSensorDown.enableLed(true);
                }

                if (gamepad1.x)

                {
                    colorSensorDown.enableLed(false);
                }

                waitFullCycle();

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

            else if (setting == START)
            {
                startPosition = (startPosition + direction) % NUM_STARTS;
                telemetryLines[START] = ("Start Position " + startPositionToString(startPosition));
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

        private String startPositionToString (int s)
        {
            switch (s)
            {
                case START_RAMP: return "RAMP START";
                case START_CORNER: return "CORNER START";
                case START_STRAIGHT: return "STRAIGHT START";
                default: return "Error: Start Position Number.";
            }
        }
    }

    public void initialize () //override
    {
        super.initialize(); //do everything in the original, common initialization.
        new ConfigLoop().start(); //
        waitFullCycle();
        colorSensorDown.enableLed(true);
    }

    public void test() //for debug, whenever we want to test something independent of the rest of the autonomous program
    {
        //move (25);
        /*
        buttonServo.setPosition(0.5);
        sleep(1000);
        buttonServo.setPosition(0.1);
        */

        colorSensorDown.enableLed(true);
        waitFullCycle();
        colorSensorDown.enableLed(true);
        waitFullCycle();

        while (runConditions())
        {
            moveWall(UP);
            setDrivePower(-0.35);
            while (runConditions() && colorSensorDown.blue() + colorSensorDown.green() >= 3)
            {

            }
            telemetry.addData("10", "on ramp");
            sleep(100);
            while (runConditions() && colorSensorDown.red() > 3)
            {

            }
            sleep(5);
            stopDrivetrain();
            moveWall(DOWN);
            telemetry.addData("10", "latched on");
            sleep(400);
            moveSwivel(SWIVEL_INIT - 0.12);
            sleep(2500);
            moveSwivel(SWIVEL_INIT);
            sleep(2500);
            setDrivePower(-0.24);
            while (runConditions() && colorSensorDown.red() > 3)
            {

            }
            moveWall(UP);
            sleep(250);
            telemetry.addData("10", "unlatched driving down");
            move(22, 0.4);
            //setDrivePower(0.6);
            /*
            while (runConditions() && colorSensorDown.green() + colorSensorDown.blue() < 3)
            {

            }

            sleep(700);
            */
            stopDrivetrain();
            telemetry.addData("10", "on floor");
            sleep(2000);
        }
    }

    public void autonomous ()
    {
        colorSensorDown.enableLed(true);
        waitFullCycle();
        colorSensorDown.enableLed(true);
        //sleep(400);
        waitFullCycle();

        if (color == BLUE)
        {
            if (startPosition == START_RAMP)
            {
                move(-11.9);
                rotateEncoder(3.6825);
                move(-27.3);
            }

            else if (startPosition == START_CORNER) //untested
            {
                move(-4);
                rotateEncoder(3.6825);
                move(-38.3);
            } else if (startPosition == START_STRAIGHT)
            {
                move (-25); //untested as of yet.
            }

            driveToLine(-0.37);
            move(-1.0);
            turnAcrossLine(0.6);
            followLineUntilTouch();
            stopDrivetrain();
            setLeftDrivePower(0);
            setRightDrivePower(0);
            flingClimbers();
            sleep(200);
            scoreRescueBeacon();
            /* for testing collecting
            if (path == LOW_GOAL_AND_COLLECT_ON_SAME_SIDE)
            {
                move(5.6);
                if (sweeperOn) setSweeperPower(1.0);
                rotateEncoder(8);
                while (runConditions())
                {
                    move(7.5);
                    move(-7.5);
                    rotateEncoder(1.94);
                    //move(5);
                }

            }
            */
        }

        else if (color == RED)
        {
            if (startPosition == START_RAMP)
            {
                move (-23);
                rotateEncoder(-6.3);
                move(-31);
            }

            else if (startPosition == START_CORNER) //untested
            {
                move (-2);
                rotateEncoder(-6.3);
                move(-42);
            }

            else if (startPosition == START_STRAIGHT) //untested
            {
                move (-25);

                setDrivePower(-0.37);
                while (runConditions() && colorSensorDown.red() < 20)
                {

                }

                stopDrivetrain();


            }

            turnToLine(-0.6);
            sleep(100);
            followLineUntilTouch();
            flingClimbers();
            sleep(200);
            scoreRescueBeacon();


        }

        setSweeperPower(0);
    }

    private void waitForLine ()
    {
        while (runConditions() && getFloorBrightness() < LINE_WHITE_THRESHOLD)
        {
            //waitFullCycle();
        }
    }

    private void driveToLine (double power)
    {
        if (!runConditions()) return;
        setDrivePower(power);
        waitForLine();
        stopDrivetrain();
        //waitFullCycle();
        //stopDrivetrain(); //one can never be too sure
    }

    private void turnToLine (double power)
    {
        if (!runConditions()) return;
        setTurnPower(power);
        waitForLine();
        stopDrivetrain();
        //waitFullCycle();
        //stopDrivetrain(); //one can never be too sure
        sleep(50);
    }

    private void turnAcrossLine (double power)
    {
        if (!runConditions()) return;
        setTurnPower(power);
        waitForLine();
        while (runConditions() && getFloorBrightness() >= LINE_WHITE_THRESHOLD)
        {
            //waitFullCycle();
        }
        stopDrivetrain();
       // waitFullCycle();
       // stopDrivetrain(); //one can never be too sure
        sleep(50);
    }

    private void followLineUntilTouch ()
    {
        if (!runConditions()) return;
        setLeftDrivePower(-0.15);

        while (runConditions() && touchSensorFront.getValue() < 0.04)
        {
            if (getFloorBrightness() < LINE_WHITE_THRESHOLD)
            {
                setRightDrivePower(0);
                setLeftDrivePower(-0.30);
            }

            else
            {
                setRightDrivePower(-0.48);
                setLeftDrivePower(0.07);
            }
            //waitFullCycle();

            if (gameTimer.time() > lineBlockedTime)
            {
                stopDrivetrain();
               // waitFullCycle();
                //stopDrivetrain();
                waitFullCycle();
                extendAndScoreClimbers();
                waitFullCycle();
                move (2);
                programFinished = true;
                return;
            }
        }

        sleep(50);
        stopDrivetrain();
        waitFullCycle();
        setLeftDrivePower(0);
        setRightDrivePower(0);
        stopDrivetrain();
        //
    }

    private void scoreRescueBeacon ()
    {
        if (!runConditions()) return;
        boolean rescueBeaconColor = getRescueBeaconColor();
        //writeToLog("Right Rescue Beacon Color: " + (rescueBeaconColor == color));
        if (rescueBeaconColor == color)
        {
            buttonServo.setPosition(0.5);
            sleep(1000);
            buttonServo.setPosition(0.1);
            sleep(500);
            move(5);
        }

        else
        {
            move(2);
            moveSwivel(SWIVEL_INIT + 0.1);
            sleep(1350);
            moveTime(1000, -0.2);
            sleep(150);
            move(2.8, 0.4);
        }
    }

    private void extendAndScoreClimbers()
    {
        slideMotor.setPower(0.6); //will be better once we have encoder on the slide motor
        sleep(600);
        slideMotor.setPower(0);
        waitFullCycle();
        flingClimbers();
        waitFullCycle();
        slideMotor.setPower(-0.6);
        sleep(600);
        slideMotor.setPower(0);
        waitFullCycle();
    }

    public void main ()
    {
        //new ProgramKiller().start(); //PROGRAM KILLER MESSES UP AUTONOMOUS.
        new DebuggerDisplayLoop().start();
        sleep(startWaitTime * 1000);
        test();
       // autonomous();
    }


}
