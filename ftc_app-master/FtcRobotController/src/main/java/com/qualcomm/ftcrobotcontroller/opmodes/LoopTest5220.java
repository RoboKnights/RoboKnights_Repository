/* Copyright (c) 2014 Qualcomm Technologies Inc

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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.util.Range;

/**
 * TeleOp Mode
 * <p>
 * Enables control of the robot via the gamepad
 */
public class LoopTest5220 extends OpMode
{

    //CONSTANTS:

    protected static final int HAS_NOT_STARTED = 0;
    protected static final int SETUP = 1;
    protected static final int INIT = 2;
    protected static final int WAITING = 3;
    protected static final int RUNNING = 4;
    //protected static enum ProgramType {UNDECIDED, AUTONOMOUS, TELEOP};

    protected static final double NORMAL = 2;
    protected static final double ENCODER = 3;
    protected static final double GYRO = 4;

    protected static final double WHEEL_DIAMETER = 6.0; //in inches
    protected static final double GEAR_RATIO = 2 / 3;
    protected static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;
    protected static final int ENCODER_COUNTS_PER_ROTATION = 1440;

    //CONFIGURABLE CONSTANTS:
    protected static final double DEFAULT_DRIVE_POWER = 0.95;
    protected static final double DEFAULT_SYNC_POWER = 0.56;
    protected static final double DEFAULT_TURN_POWER = 0.30;
    protected static final double DEFAULT_TURN_POWER_HIGH = 0.80;
    protected static final double INIT_SERVO_POSITION = 0.5;

    protected static final double ENCODER_SYNC_PROPORTIONALITY_CONSTANT = 0.001; //0.001 means 50 encoder counts --> 5% power difference
    protected static final double GYRO_SYNC_PROPORTIONALITY_CONSTANT = 0.02; //this times 100 is the motor power difference per degree off.
    protected static final double ENCODER_SYNC_UPDATE_TIME = 20; //in milliseconds for convenience
    protected static final double GYRO_SYNC_UPDATE_TIME = 20; //in milliseconds for convenience

    //MOTORS AND SERVOS:

    protected static final String[] motorNames = {}; //Fill this in later.

    protected DcMotorController driveController1; //MAKE SURE THESE THINGS HAVE SAME NAME AS IN PHONE CONFIGURATION
    protected DcMotorController driveController2;
    protected DcMotorController armAndSweeperController;
    protected ServoController armServoController;

    protected DcMotor leftFrontMotor;
    protected DcMotor rightFrontMotor;
    protected DcMotor leftBackMotor;
    protected DcMotor rightBackMotor;
    protected DcMotor sweeperMotor;
    //protected DcMotor armMotor;

    protected DcMotor[] driveMotors = new DcMotor[4];

    protected Servo swivelServo;
    protected Servo armServo;
    protected Servo tiltServo;

    //OTHER GLOBAL VARIABLES:

    protected Stopwatch gameTimer;
    protected int phase = HAS_NOT_STARTED;

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

    public void setup()//this and the declarations above are the equivalent of the pragmas in RobotC
    {
        phase = SETUP;

        driveController1 = hardwareMap.dcMotorController.get("Motor Controller 1");
        driveController1.setMotorChannelMode(1, DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        driveController1.setMotorChannelMode(2, DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);

        driveController2 = hardwareMap.dcMotorController.get("Motor Controller 2");
        driveController2.setMotorChannelMode(1, DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        driveController2.setMotorChannelMode(2, DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);

        armAndSweeperController = hardwareMap.dcMotorController.get("Motor Controller 3");
        armAndSweeperController.setMotorChannelMode(1, DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        armAndSweeperController.setMotorChannelMode(2, DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);

        armServoController = hardwareMap.servoController.get("Servo Controller 4");

        leftFrontMotor = hardwareMap.dcMotor.get("lf");
        rightFrontMotor = hardwareMap.dcMotor.get("rf");
        leftBackMotor = hardwareMap.dcMotor.get("lb");
        rightBackMotor = hardwareMap.dcMotor.get("rb");

        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        leftBackMotor.setDirection(DcMotor.Direction.REVERSE);

        driveMotors[0] = leftFrontMotor;
        driveMotors[1] = rightFrontMotor;
        driveMotors[2] = leftBackMotor;
        driveMotors[3] = rightBackMotor;

        for (DcMotor dcm: driveMotors) dcm.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
        for (DcMotor dcm: driveMotors) dcm.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);

        // armMotor = hardwareMap.dcMotor.get("arm");
        sweeperMotor = hardwareMap.dcMotor.get("sweeper");

        swivelServo = hardwareMap.servo.get("sServo");
        armServo = hardwareMap.servo.get("rServo");
        tiltServo = hardwareMap.servo.get("tServo");

        gameTimer = new Stopwatch();

        /*
        servoL = hardwareMap.servo.get("servo_P1_1");
        servoR = hardwareMap.servo.get("servo_P1_2");
        */
    }

    public class DebuggerDisplayLoop extends Thread
    {
        public void run()
        {
            while (true)
            {
                telemetry.addData("1", "LFM: " + leftFrontMotor.getCurrentPosition());
                telemetry.addData("2", "RFM: " + rightFrontMotor.getCurrentPosition());
                telemetry.addData("3", "LBM: " + leftBackMotor.getCurrentPosition());
                telemetry.addData("4", "RBM: " + rightBackMotor.getCurrentPosition());
                telemetry.addData("5", "Swivel: " + swivelServo.getPosition());
                telemetry.addData("6", "Arm: " + armServo.getPosition());
                telemetry.addData("7", "Tilt: " + tiltServo.getPosition());
                telemetry.addData("8", "Time Elapsed:" + gameTimer.time());
            }
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

    public int getEncoderValue (DcMotor motor)
    {
        return motor.getCurrentPosition();
    }

    public double getGyroDirection () //placeholder
    {
        return 1.0;
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
        for (DcMotor motor: driveMotors)
        {
            motor.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
            motor.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        }

    }

    public final boolean driveEncodersHaveReached(int encoderCount)
    {
        if (Math.abs(leftBackMotor.getCurrentPosition()) > encoderCount)
        {
            telemetry.addData("9", "" + leftFrontMotor.getCurrentPosition());
            return true;


        }

        else
        {
            return false;
        }
    }

    public final boolean turnEncodersHaveReached(int count)
    {
        return false;
    }

    public final void move (double distance, double... params)
    {
        double power = DEFAULT_DRIVE_POWER;
        double mode = NORMAL;

        if (params.length > 2)
        {
            return;
        }

        else if (params.length == 1)
        {
            if (Math.abs(params[0]) < 1.1) //second parameter is power, 1.1 used instead of 1 to completely deal with floating point inaccuracy
            {
                power = params[0];
            }

            else
            {
                mode = params[0];
                if (mode != NORMAL)
                {
                    power = DEFAULT_SYNC_POWER;
                }
            }
        }

        else if (params.length == 2)
        {
            power = params[0];
            mode = params[1];
        }

        //If params.length == 0, no need to do anything, since power and mode are initialized to their proper values.

        //Main method body:

        int encoderCount = distanceToEncoderCount(distance);
        double initialDirection = getGyroDirection();

        double powerChange = 0;
        double updateTime = ((mode == ENCODER) ? ENCODER_SYNC_UPDATE_TIME : GYRO_SYNC_UPDATE_TIME) / 1000;

        resetDriveEncoders();
        setDrivePower(power);

        while (!driveEncodersHaveReached(encoderCount))
        {
            if (mode != NORMAL)
            {
                if (mode == ENCODER)
                {
                    double frontDifference = getEncoderValue(leftFrontMotor) - getEncoderValue(rightFrontMotor);
                    double backDifference = getEncoderValue(leftBackMotor) - getEncoderValue(rightBackMotor);
                    double averageDifference = (frontDifference + backDifference) / 2;
                    powerChange = averageDifference * ENCODER_SYNC_PROPORTIONALITY_CONSTANT;
                }

                else if (mode == GYRO)
                {
                    powerChange = (getGyroDirection() - initialDirection) * GYRO_SYNC_PROPORTIONALITY_CONSTANT;
                }

                setLeftDrivePower(power - powerChange);
                setRightDrivePower(power + powerChange);

                double initTime = gameTimer.time();
                while ((gameTimer.time() - initTime) < updateTime)
                {
                    if (driveEncodersHaveReached(encoderCount))
                    {
                        break;
                    }
                }
            }

            //do nothing if mode is NORMAL.
        }

        stopDrivetrain();
    }

    public void moveSimple (int count)
    {
        setDrivePower(DEFAULT_DRIVE_POWER);
        while (!driveEncodersHaveReached(count))
        {

        }
        stopDrivetrain();
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

    //ATTACHMENTS:

    public static final boolean COLLECT = false;
    public static final boolean DISPENSE = true;

    private boolean armPosition = COLLECT;

    public final void moveArm ()
    {
        armPosition = !armPosition;

        if (armPosition == COLLECT) {

        } else if (armPosition == DISPENSE) {

        }
    }

	/*
	 * Code to run when the op mode is first enabled goes here
	 * 
	 * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
	 */
	@Override
	public void init()
    {
        setup();
       // new DebuggerDisplayLoop().start();
	}

	/*
	 * This method will be called repeatedly in a loop
	 * 
	 * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#run()
	 */
	//@Override

    private static final double JOYSTICK_THRESHOLD = 0.08; //below this joysticks won't cause movement.

    private double g1Stick1Xinit;
    private double g1Stick1Yinit;
	public void loop()
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

        // telemetry.addData("2", "Left: " + left);
        // telemetry.addData("3", "Right: " + right);

        setLeftDrivePower(left);
        setRightDrivePower(right);

        if (gamepad1.left_stick_button)
        {
            g1Stick1Xinit = gamepad1.left_stick_x;
            g1Stick1Yinit = gamepad1.left_stick_y;
        }

        setMotorPower(sweeperMotor, (gamepad1.right_bumper ? 0.99 : 0));

        tiltServo.setPosition(gamepad1.a ? 0.99 : 0);
        armServo.setPosition(1 - Math.abs(gamepad1.right_stick_y));
        swivelServo.setPosition(0.76); //full range is 6.25 rotation, approximately. 1 is collection position

        telemetry.addData("1", "LFM: " + leftFrontMotor.getCurrentPosition());
        telemetry.addData("2", "RFM: " + rightFrontMotor.getCurrentPosition());
        telemetry.addData("3", "LBM: " + leftBackMotor.getCurrentPosition());
        telemetry.addData("4", "RBM: " + rightBackMotor.getCurrentPosition());
        telemetry.addData("5", "Swivel: " + swivelServo.getPosition());
        telemetry.addData("6", "Arm: " + armServo.getPosition());
        telemetry.addData("7", "Tilt: " + tiltServo.getPosition());
        telemetry.addData("8", "Time Elapsed:" + gameTimer.time());

	}

	/*
	 * Code to run when the op mode is first disabled goes here
	 * 
	 * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#stop()
	 */
	@Override
	public void stop() {

	}
}
