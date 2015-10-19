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

//add program off switch for using any one particular motor

public abstract class OpMode_5220 extends LinearOpMode //FIGURE OUT HOW TO GET DECENT VERSION CONTROL INSTEAD OF HAVING TO SAVE AS FOR EVERY NEW VERSION
{
    //CONSTANTS:

    protected static final int HAS_NOT_STARTED = 0;
    protected static final int SETUP = 1;
    protected static final int INIT = 2;
    protected static final int WAITING = 3;
    protected static final int RUNNING = 4;
    //protected static enum ProgramType {UNDECIDED, AUTONOMOUS, TELEOP};

    protected static final double WHEEL_DIAMETER = 6.0; //in inches
    protected static final double GEAR_RATIO = 2 / 3;
    protected static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;
    protected static final int ENCODER_COUNTS_PER_ROTATION = 1440;

    //CONFIGURABLE CONSTANTS:
    protected static final double DEFAULT_DRIVE_POWER = 0.95;
    protected static final double DEFAULT_TURN_POWER = 0.30;
    protected static final double DEFAULT_TURN_POWER_HIGH = 0.80;
    protected static final double INIT_SERVO_POSITION = 0.5;

    //MOTORS AND SERVOS:
    protected DcMotorController driveController1; //MAKE SURE THESE THINGS HAVE SAME NAME AS IN PHONE CONFIGURATION
    protected DcMotorController driveController2;
    protected DcMotor leftFrontMotor;
    protected DcMotor rightFrontMotor;
    protected DcMotor leftBackMotor;
    protected DcMotor rightBackMotor;

    //OTHER GLOBAL VARIABLES:

    protected Stopwatch gameTimer;
    protected int phase = HAS_NOT_STARTED;

    public void setup()//this and the declarations above are the equivalent of the pragmas in RobotC
    {
        phase = SETUP;

        driveController1 = hardwareMap.dcMotorController.get("motorController_P0");
        driveController1.setMotorChannelMode(1, DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        driveController1.setMotorChannelMode(2, DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);

        driveController2 = hardwareMap.dcMotorController.get("motorController_P1");
        driveController2.setMotorChannelMode(1, DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        driveController2.setMotorChannelMode(2, DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);

        leftFrontMotor = hardwareMap.dcMotor.get("motor_P0_1");
        rightFrontMotor = hardwareMap.dcMotor.get("motor_P0_2");
        leftBackMotor = hardwareMap.dcMotor.get("motor_P1_1");
        rightBackMotor = hardwareMap.dcMotor.get("motor_P1_2");

        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        leftBackMotor.setDirection(DcMotor.Direction.REVERSE);
        /*
        servoL = hardwareMap.servo.get("servo_P1_1");
        servoR = hardwareMap.servo.get("servo_P1_2");
        */
    }

    public void initialize()
    {
        phase = INIT;
    }

    public void waitForStart5220() //override this to do stuff if neccessary
    {
        while (!opModeIsActive())
        {

        }
    }

    public void waitForStart () throws InterruptedException //forget about this override and put the phase change in runOpMode if neccessary.
    {
        phase = WAITING;
        super.waitForStart();
    }

    public abstract void main(); //implement in all subclasses. This is the main body of the program. Maybe also make initializeRobot something to override if its different between OpModes.

    public final void runOpMode() throws InterruptedException
    {
        setup();
        initialize();
        //waitForStart5220(); //uncomment this if it turns out that it works just as well as waitForStart();
        waitForStart();

        phase = RUNNING;
        gameTimer = new Stopwatch();

        main();
    }

    //HELPER CLASSES AND METHODS:
    //______________________________________________________________________________________________________________
    public class Stopwatch
    {
        private final long start;

        public Stopwatch() {
            start = System.currentTimeMillis();
        }

        public double time() {
            long now = System.currentTimeMillis();
            return ((double) (now - start)) / 1000.0;
        }
    }

    public int distanceToEncoderCount (double distance) //distance is in inches
    {
        double wheelRotations = distance / WHEEL_CIRCUMFERENCE;
        double motorRotations = wheelRotations / GEAR_RATIO;
        long encoderCounts = Math.round(motorRotations * ENCODER_COUNTS_PER_ROTATION);
        return (int) encoderCounts; //typecast is okay because the encoder count should NEVER exceed Integer.MAX
    }

    public void updateLoopBody()
    {

    }

    public void sleep(int millis)
    {
        try
        {
            Thread.sleep(millis);
        }

        catch (Exception e)
        {
            System.exit(0);
        }
    }

    public void update_telemetry() //fix this to make it actually useful later. or maybe let is override
    {
        telemetry.addData("01", "Hello world!");
    }

    //MOVEMENT:

    //changes to make: unify moveTime and move with encoder into method with choice of encoder or time as a parameter.
    //do the same thing for rotation except with three options: gyro, encoder, and time.

    public final void setMotorPower(DcMotor motor, double power) //maybe not neccessary
    {
        motor.setPower(power);
    }

    public final void setLeftDrivePower (double power)
    {
        setMotorPower (leftFrontMotor, power);
        setMotorPower (leftBackMotor, power);
    }

    public final void setRightDrivePower (double power)
    {
        setMotorPower (rightFrontMotor, power);
        setMotorPower (rightBackMotor, power);
    }

    public final void setDrivePower (double power)
    {
        setLeftDrivePower(power);
        setRightDrivePower(power);
    }
    


    public final void stopDrivetrain ()
    {
        setDrivePower(0);
    }

    public final void resetDriveEncoders ()
    {

    }

    public final boolean driveEncodersHaveReached(int count)
    {
        return false;
    }
    public final boolean turnEncodersHaveReached(int count)
    {
        return false;
    }

    public final void move(double distance, double power) //add something to make sure that negative distance = negative power.
    {
        resetDriveEncoders();
        setDrivePower(power);
        while (!driveEncodersHaveReached(distanceToEncoderCount(distance)))
        {

        }
        stopDrivetrain();
    }

    public final void move(double distance)
    {
        move (distance, DEFAULT_DRIVE_POWER);
    }

    public final void moveTime(int time, double power)
    {
        setDrivePower(power);
        sleep(time);
        stopDrivetrain();
    }

    /* fix this so that negative time means go backwards
    public final void moveTime(int time) //time is in millis
    {
        moveTime (time, DEFAULT_DRIVE_POWER);
    }
*/
    //ROTATION:

    public final void setTurnPower (double power)
    {
        setLeftDrivePower(power);
        setRightDrivePower(-power);
    }

    public final void waitForGyroRotation (double degrees)
    {
        //finish later
    }

    public final void rotate (double degrees, double power) //gyro rotation, add thing to make negative degrees = negative power.
    {
        setTurnPower(power);
        waitForGyroRotation (degrees);
        stopDrivetrain();
    }

    public final void rotate (double degrees)
    {
        rotate (degrees, DEFAULT_TURN_POWER);
    }

    public final void rotateEncoder (double distance, double power) //add thing to make negative distance = negative power.
    {
        resetDriveEncoders();
        setTurnPower(power);
        while (!turnEncodersHaveReached(distanceToEncoderCount(distance)))
        {

        }
        stopDrivetrain();
    }

    public final void rotateEncoder (double distance)
    {
        rotateEncoder(distance, DEFAULT_TURN_POWER_HIGH);
    }

    public final void rotateTime (int time, double power) //time in millis
    {
        setTurnPower(power);
        sleep (time);
        stopDrivetrain();
    }
}