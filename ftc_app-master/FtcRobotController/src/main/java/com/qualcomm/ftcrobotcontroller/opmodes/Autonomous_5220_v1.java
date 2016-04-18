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
import com.qualcomm.ftcrobotcontroller.R;
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
    public static final int PARK = 0;
    public static final int COLLECTION = 1;
    public static final int RAMP = 2;
    public static final int OTHER_SIDE = 3;
    public static final int CLEAR = 4;
    public static final int BACK_CLEAR = 5;
    public static final int NUM_PATHS = 6;

    public static final int START_RAMP = 0;
    public static final int START_CORNER = 1;
    public static final int START_STRAIGHT = 2;
    public static final int NUM_STARTS = 3;

    public double lineBlockedTime = 15500;
    private boolean lineBlocked = false;

    public static final int OFF_RAMP_STALL_TIME = 4000;
    public static final int ON_RAMP_STALL_TIME = 3500;

    private Autonomous_5220_v1 opMode = this;

    private boolean color = RED; //arbitrary default
    private int startPosition = START_RAMP;
    private int path = PARK;
    private int startWaitTime = 0; //in seconds, no need for non-integer numbers.
    private boolean beaconScoringOn = true;
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
        private static final int PATH = 3;
        private static final int BEACON = 4;
        private static final int SWEEP = 5;


        private static final int NUM_SETTINGS = 6; //always make sure this is correct.

        private int currentSetting = 0;

        private String[] telemetryLines = new String[NUM_SETTINGS];


        public void run ()
        {
            for (int i = 0; i < telemetryLines.length; i++) telemetryLines[i] = "";
            telemetryLines[COLOR] = ("Color: " + (color == RED ? "RED" : "BLUE")); //maybe add starter asterisk here. Not sure if it is neccessary.
            telemetryLines[START] = ("Start Position " + (startPosition == START_RAMP ? "RAMP" : "CORNER"));
            telemetryLines[WAIT] = ("Wait Time (in seconds): " + startWaitTime /*+ " seconds"*/);
            telemetryLines[BEACON] = ("Beacon Scoring: " + (beaconScoringOn ? "ON" : "OFF"));
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

            else if (setting == BEACON)
            {
                beaconScoringOn = !beaconScoringOn;

                telemetryLines[BEACON] = ("Beacon Scoring: " + (beaconScoringOn ? "ON" : "OFF"));
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
                case PARK: return "PARK";
                case COLLECTION: return "COLLECTION";
                case RAMP: return "RAMP";
                case OTHER_SIDE: return "OTHER SIDE";
                case CLEAR: return "CLEAR";
                case BACK_CLEAR: return "BACK CLEAR";
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

    private class HookRetractor extends Thread
    {
        public void run ()
        {
            setLiftPower(0.2);
            opMode.sleep(1400);
            setLiftPower(0);
            setLiftPower(0);
        }
    }

    public void test() //for debug, whenever we want to test something independent of the rest of the autonomous program
    {
        telemetry.addData("1", "Started moving.");
        rotateIMU(-180);
        sleep(2500);
        rotateIMU(180);
        telemetry.addData("1", "Done moving..");
    }

    public void autonomous ()
    {
        //new HookRetractor().start();
        startToLine(color);

        followLineUntilTouch();

        if (lineBlocked)
        {
            extendAndScoreClimbers();
        }

        else
        {
            flingClimbers();
            sleep(200);
            if (beaconScoringOn)scoreRescueBeacon();
        }

        if (path == BACK_CLEAR)
        {
            if (color == BLUE)
            {
                move(42);
                while (gameTimer.time() < 27000);
                moveTime(30000 - gameTimer.time() - 400, 0.10);
                stopDrivetrain();
            }

            if (color == RED)
            {
                move(42);
                while (gameTimer.time() < 27000);
                moveTime(30000 - gameTimer.time() - 400, 0.10);
                stopDrivetrain();
            }
        }

        if (path == CLEAR)
        {
            if (color == BLUE)
            {
                move(3);
                rotateEncoder(-14, 0.5);
                //swingTurn(28, RIGHT, 0.7);
                move(-27);
                while (gameTimer.time() < 27000);
                moveTime(30000 - gameTimer.time() - 400, 0.10);
                stopDrivetrain();
            }

            if (color == RED)
            {
                move(3);
                rotateEncoder(13.2, 0.5);
                //swingTurn(28, RIGHT, 0.7);
                move(-31.15);
                while (gameTimer.time() < 27000);
                moveTime(30000 - gameTimer.time() - 400, 0.10);
                stopDrivetrain();
            }

            return;
        }

        setDrivePower(0.36);
        waitForOnlyAllianceLine();
        stopDrivetrain();
        stopDrivetrain();

        //At this point, robot should straight towards the wall, with color sensor directly above the colored line

        if (path == PARK)
        {
            move(-5.6);
            playMusic(R.raw.all_i_do_is_win);
        }

        else if (path == COLLECTION)
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

        else if (path == RAMP)
        {
            driveToRamp();
            climbRamp();
            stopDrivetrain(); //neccessary, otherwise would have to put this in 4 different places in climbRamp.
        }

        else if (path == OTHER_SIDE)
        {
            if (color == RED)
            {
                rotateEncoder(27, 0.6);
            }
            else if (color == BLUE)
            {
                rotateEncoder(-29.5);
            }
            move(-77);

        }

        setSweeperPower(0);
    }

    private void startToLine (boolean c)
    {
        if (c == BLUE)
        {
            if (startPosition == START_RAMP)
            {/*
                move(-11.9);
                rotateEncoder(2.392);
                move(-24.5);
                driveToLine(-0.37);
                move(1, 0.4);
                straightenWithLine();
                */
                move (-29.5, ENCODER);
                rotateEncoder(7.45);
                move(-46, ENCODER);
                driveToLine(-0.24);
                move(-2.0, 0.14);
                turnAcrossLine(0.7);
                turnToLine(-0.25);
            }

            else if (startPosition == START_CORNER) //untested
            {
                /*
                move(-46);
                rotateEncoder(3.1);
                move(-17);
                rotateEncoder(-6);
                move (-10);
                driveToLine(-0.37);
                move(1, 0.4);
                straightenWithLine();
                */
                /*
                move (-69);
                rotateEncoder(3.1);
                sleep(400);
                setDrivePower(-0.5);
                waitForAllianceLine();
                stopDrivetrain();
                move(-1.1, 0.3);
                rotateEncoder(-8.2);
                move(3.2, 0.3);
                driveToLine(-0.3);
                move(1, 0.3);
                straightenWithLine();
                */

                move (-84, ENCODER);
                rotateEncoder(7.0, 0.5);
                move(-21, ENCODER);
                rotateEncoder(-8.0, 0.5);
                driveToLine(-0.24);
                move(-2.0, 0.14);
                turnAcrossLine(0.7);
                turnToLine(-0.25);
            }

            else if (startPosition == START_STRAIGHT)
            {
                move (-25); //untested as of yet.
                driveToLine(-0.37);
                move(1, 0.4);
                straightenWithLine();
            }
        }

        else if (c == RED)
        {

            if (startPosition == START_RAMP)
            {
                /*
                move (-48.5, ENCODER);
                rotateEncoder(-9);
                move(-34.1, ENCODER);
                rotateEncoder(-7.3);
                setDrivePower(-0.4);
                waitForAllianceLine();
                stopDrivetrain();
                move(-1.1, 0.3);
                rotateEncoder(-12.8);
                move(5.1, 0.1);
                driveToLine(-0.3);
                move(1, 0.3);
                straightenWithLine();
                */
                move (-49, ENCODER);
                rotateEncoder(-9.5);
                move(-42.1, ENCODER);
                sleep(250);
                waitFullCycle();
                rotateEncoder(-12);
                sleep(250);
                waitFullCycle();
                driveToLine(-0.1);
                waitFullCycle();
                move(-2.1, 0.14);
                turnAcrossLine(0.7);
                turnToLine(-0.25);
               // rotateEncoder(3.89);
                //turnAcrossLine (0.6);
            }

            else if (startPosition == START_CORNER)
            {
                /*
                move (-3);
                sleep (200);
                rotateEncoder(-5.2);
                move(-63);
                rotateEncoder(-12);
                driveToLine(-0.3);
                move (-0.8);
                turnAcrossLine (0.6);
                */
                //move (-5);
               // rotateTime(180, -0.7);

                /* THIS WORKS BUT SWITCHED TO NEW THING
                move (-77.7);
                rotateEncoder(-11.3);
                driveToLine(-0.37);
                move (1.1);
                straightenWithLine();
                */
/*
                move (-77);
                rotateEncoder(-6.1);
                sleep(400);
                setDrivePower(-0.4);
                waitForAllianceLine();
                stopDrivetrain();
                move(-1.1, 0.3);
                rotateEncoder(-8.2);
                move(5.1, 0.3);
                driveToLine(-0.3);
                move(1, 0.3);
                straightenWithLine();

                */

                move (-109.64, ENCODER);
                sleep(250);
                waitFullCycle();
                rotateEncoder(-14.15, 0.5);
                sleep(250);
                waitFullCycle();
                driveToLine(-0.24);
                waitFullCycle();
                move(-2.0, 0.14);
                turnAcrossLine(0.7);
                turnToLine(-0.25);

            }

            else if (startPosition == START_STRAIGHT) //untested
            {
                move (-25);



                setDrivePower(-0.37);
                waitForAllianceLine();

                stopDrivetrain();
                rotateEncoder(-12);
                driveToLine(0.3);
                turnAcrossLine(0.7);
            }

            //turnToLine(-0.6);
        }

        sleep(100);
    }

    private void straightenWithLine () //need to test this
    {
        //Stopwatch darkTime = new Stopwatch();
        driveToLine(-0.1);
        //int gyroInit = getGyroDirection();
        int encInit = leftFrontMotor.getCurrentPosition();
        boolean lastWasWhite = false;
        while (runConditions())
        {
            if (touchSensorFront.getValue() > 0.04) break;

            if (getFloorBrightness() > LINE_WHITE_THRESHOLD)
            {
                lastWasWhite = true;

                setLeftDrivePower(0.28);
                setRightDrivePower(-0.03);
            }

            else
            {
                if (lastWasWhite)
                {
                    encInit = leftFrontMotor.getCurrentPosition();
                    lastWasWhite = false;
                }
                if (Math.abs(leftFrontMotor.getCurrentPosition() - encInit) > 160) break;

                setLeftDrivePower(-0.1);
                setRightDrivePower(-0.1);
            }
        }

        stopDrivetrain();
    }

    private void waitForAllianceLine ()
    {
        waitForColoredLine(color);
    }

    private void waitForOnlyAllianceLine ()
    {
        waitForOnlyColoredLine(color);
    }

    private void waitForOnlyColoredLine (boolean c)
    {
        while (runConditions())
        {
            if (c == RED)
            {
                if (colorSensorDown.red() > 12 && colorSensorDown.blue() < 9) break;
            }

            else if (c == BLUE)
            {
                if (colorSensorDown.blue() > 12 && colorSensorDown.red() < 9) break;
            }
        }
    }

    private void waitForColoredLine (boolean c)
    {
        while (runConditions())
        {
            if (c == RED)
            {
                if (colorSensorDown.red() > 13) break;
            }

            else if (c == BLUE)
            {
                if (colorSensorDown.blue() > 13) break;
            }
        }
    }

    private void waitForLine ()
    {
        while (runConditions() && getFloorBrightness() < LINE_WHITE_THRESHOLD)
        {

        }
    }

    private void driveToLine (double power)
    {
        if (!runConditions()) return;
        setDrivePower(power);
        waitForLine();
        stopDrivetrain();
    }

    private void turnToLine (double power)
    {
        if (!runConditions()) return;
        setTurnPower(power);
        waitForLine();
        stopDrivetrain();
        sleep(50);
    }

    private void turnAcrossLine (double power)
    {
        if (!runConditions()) return;
        setTurnPower(power);
        waitForLine();
        while (runConditions() && getFloorBrightness() >= LINE_WHITE_THRESHOLD)
        {

        }
        stopDrivetrain();
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
                setRightDrivePower(0.02);
                setLeftDrivePower(-0.22);
            }

            else
            {
                setRightDrivePower(-0.22);
                setLeftDrivePower(0.02);
            }

            if (gameTimer.time() > lineBlockedTime)
            {
                stopDrivetrain();
                lineBlocked = true;
                /*
                extendAndScoreClimbers();
                waitFullCycle();
                move (2);
                programFinished = true;
                */
                return;
            }
        }

        sleep(50);
        stopDrivetrain();
        stopDrivetrain();
        //
    }

    private void scoreRescueBeacon ()
    {
        if (!runConditions()) return;
        Boolean rbcWrapper = getRescueBeaconColor();
        if (rbcWrapper == null) return;
        //writeToLog("Right Rescue Beacon Color: " + (rescueBeaconColor == color));
        boolean rescueBeaconColor = Boolean.valueOf(rbcWrapper); //make sure this works properly
        if (rescueBeaconColor == color)
        {
            buttonServo.setPosition(0.5);
            sleep(1000);
            buttonServo.setPosition(0.1);
            sleep(500);
            //move(5);
        }

        else
        {
            move(2);
            moveSwivel(SWIVEL_INIT + 0.1);
            sleep(1350);
            moveTime(1350, -0.12);
            sleep(150);
            move(1.0, 0.4);
            moveSwivel(SWIVEL_INIT);
            /*
            setDrivePower(-0.2);
            while (touchSensorFront.getValue() < 0.04)
            {

            }
            stopDrivetrain();
            */
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

    private void driveToRamp () //untested
    {
        if (color == RED)
        {
            rotateEncoder(-17.3); //CHANGE BACK TO WHAT IT WAS BEFORE, SOON.
            move(-15);
            rotateEncoder(3.9);
        }

        else if (color == BLUE)
        {
            rotateEncoder(8);
            move(-15);
            rotateEncoder(4);
        }

    }

    private void climbRamp () //blue is untested
    {
        moveWall(UP);
        int rampStartTime = gameTimer.time();
        setDrivePower(-0.35);
        if (color == RED)
        {
            while (runConditions() && colorSensorDown.blue() + colorSensorDown.green() >= 3)
            {
                if (gameTimer.time() > rampStartTime + OFF_RAMP_STALL_TIME) return;
            }
        }

        else if (color == BLUE)
        {
            while (runConditions() && colorSensorDown.red() + colorSensorDown.green() >= 3)
            {
                if (gameTimer.time() > rampStartTime + OFF_RAMP_STALL_TIME) return;
            }
        }

        rampStartTime = gameTimer.time();
        sleep(100);
        if (color == RED)
        {
            while (runConditions() && colorSensorDown.red() > 3)
            {
                if (gameTimer.time() > rampStartTime + ON_RAMP_STALL_TIME) return;
            }
        }

        else if (color == BLUE)
        {
            while (runConditions() && colorSensorDown.blue() > 3)
            {
                if (gameTimer.time() > rampStartTime + ON_RAMP_STALL_TIME) return;

            }
        }
        sleep(5);
        stopDrivetrain();
        moveWall(DOWN);
    }

    public Boolean getRescueBeaconColor () //experimental for the time being
    {
        int red = colorSensorFront.red();
        int blue = colorSensorFront.blue();
        int green = colorSensorFront.green();

        if (red > blue) return RED;
        else if (blue > red) return BLUE;
        else return null;
    }

    public void main ()
    {
        //new ProgramKiller().start(); //PROGRAM KILLER MESSES UP AUTONOMOUS.
        new DebuggerDisplayLoop().start();
        waitFullCycle();

        navX.zeroYaw();
        waitFullCycle();

        lineBlockedTime = lineBlockedTime + startWaitTime;
        if (startPosition == START_CORNER) lineBlockedTime = lineBlockedTime + 12; //tiny value is intentional, blue is about as fast as red.

        colorSensorDown.enableLed(true);
        waitFullCycle();
        colorSensorDown.enableLed(true);
        waitFullCycle();

        while (gameTimer.time() < (startWaitTime * 1000))
        {

        }
        test();
        //autonomous();
        stopMusic();
    }
}
