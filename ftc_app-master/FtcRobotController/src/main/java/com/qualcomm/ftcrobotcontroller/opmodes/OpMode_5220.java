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
    protected static final int DEFAULT_DRIVE_POWER = 95;
    protected static final double INIT_SERVO_POSITION = 0.5;

    //MOTORS AND SERVOS:
    protected DcMotorController driveController1; //MAKE SURE THESE THINGS HAVE SAME NAME AS IN PHONE CONFIGURATION
    protected DcMotorController driveController2;
    protected DcMotor leftFrontMotor;
    protected DcMotor rightFrontMotor;
    protected DcMotor leftBackMotor;
    protected DcMotor rightBackMotor;

    //OTHER GLOBAL VARIABLES:

    Stopwatch gameTimer;

    public final void setup()//this and the declarations above are the equivalent of the pragmas in RobotC
    {
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
        /*
        servoL = hardwareMap.servo.get("servo_P1_1");
        servoR = hardwareMap.servo.get("servo_P1_2");
        */
        specialSetup();
    }

    public void specialSetup ()
    {

    }

    public final void initialize()
    {
        specialInitialize();
    }

    public void specialInitialize () //not sure whether to keep this whole "special" system or not
    {

    }

    public abstract void main(); //implement in all subclasses. This is the main body of the program. Maybe also make initializeRobot something to override if its different between OpModes.

    public final void runOpMode() throws InterruptedException
    {
        setup();
        initialize();
        waitForStart();
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
            return (now - start) / 1000.0;
        }
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
    
    public final void setTurnPower (double power)
    {
        setLeftDrivePower(power);
        setRightDrivePower(-power);
    }

    public final void stopDrivetrain ()
    {
        setDrivePower(0);
    }

    public final void move(int encoderCount) {

    }

    public final void move(int encoderCount, double power)
    {

    }

    public final void moveTime(int time, double power)
    {
        setDrivePower(power);
        sleep(time);
        stopDrivetrain();
    }

    public final void moveTime(int time) //time is in millis
    {
        moveTime (time, DEFAULT_DRIVE_POWER);
    }
}