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
//hello!

//add program off switch for using any one particular motor

public abstract class OpMode_5220 extends LinearOpMode //FIGURE OUT HOW TO GET DECENT VERSION CONTROL INSTEAD OF HAVING TO SAVE AS FOR EVERY NEW VERSION
{
    //CONSTANTS:

    protected static final int HAS_NOT_STARTED = 0;
    protected static final int SETUP = 1;
    protected static final int INIT = 2;
    protected static final int WAITING = 3;
    protected static final int RUNNING = 4;

    protected static enum ProgramType {UNDECIDED, AUTONOMOUS, TELEOP};
    protected static ProgramType programType = ProgramType.UNDECIDED;

    protected static final double NORMAL = 2;
    protected static final double ENCODER = 3;
    protected static final double GYRO = 4;

    protected static final double WHEEL_DIAMETER = 6.0; //in inches
    protected static final double GEAR_RATIO = 2 / 3;
    protected static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;
    protected static final int ENCODER_COUNTS_PER_ROTATION = 1440;

    //CONFIGURABLE CONSTANTS:

    protected static final boolean TIMER_ON = false;
    protected static final int TIMER_STOP_BUFFER = 500; //in millis

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
    protected DcMotor hookMotor;
    //protected DcMotor armMotor;

    protected DcMotor[] driveMotors = new DcMotor[4];

    protected Servo swivelServo;
    protected Servo armServo;
    protected Servo doorServo;
    protected Servo hookTiltServo;

    protected double swivelServoInit;

    //SENSORS:
    protected ColorSensor colorSensor;

    //OTHER GLOBAL VARIABLES:

    protected Stopwatch gameTimer;
    protected int phase = HAS_NOT_STARTED;

    public void setup()//this and the declarations above are the equivalent of the pragmas in RobotC
    {
        phase = SETUP;

        driveController1 = hardwareMap.dcMotorController.get("Motor Controller 1");
        //driveController1.setMotorChannelMode(1, DcMotorController.RunMode.RUN_USING_ENCODERS);
        //driveController1.setMotorChannelMode(2, DcMotorController.RunMode.RUN_USING_ENCODERS);

        driveController2 = hardwareMap.dcMotorController.get("Motor Controller 2");
       // driveController2.setMotorChannelMode(1, DcMotorController.RunMode.RUN_USING_ENCODERS);
       // driveController2.setMotorChannelMode(2, DcMotorController.RunMode.RUN_USING_ENCODERS);

        armAndSweeperController = hardwareMap.dcMotorController.get("Motor Controller 3");
        //armAndSweeperController.setMotorChannelMode(1, DcMotorController.RunMode.RUN_USING_ENCODERS);
       // armAndSweeperController.setMotorChannelMode(2, DcMotorController.RunMode.RUN_USING_ENCODERS);

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
        hookMotor = hardwareMap.dcMotor.get("hook");

        hookMotor.setDirection(DcMotor.Direction.REVERSE);

        swivelServo = hardwareMap.servo.get("sServo");
        armServo = hardwareMap.servo.get("aServo");
        doorServo = hardwareMap.servo.get("dServo");
        hookTiltServo = hardwareMap.servo.get("hServo");

        colorSensor = hardwareMap.colorSensor.get("cSensor1");

        /*
        servoL = hardwareMap.servo.get("servo_P1_1");
        servoR = hardwareMap.servo.get("servo_P1_2");
        */
    }

    public void initialize()
    {
        //swivelServoInit = swivelServo.getPosition();
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
        end();
    }

    public void end()
    {
       stopDrivetrain();
    }

    //HELPER CLASSES AND METHODS:
    //______________________________________________________________________________________________________________
    public class Stopwatch
    {
        private final long start;

        public Stopwatch() {
            start = System.currentTimeMillis();
        }

        public int time()
        {
            long now = System.currentTimeMillis();
            return ((int) (now - start));
        }

        public double timeSeconds()
        {
            long now = System.currentTimeMillis();
            return (((double) (now - start)) / 1000);
        }
    }

    public class DebuggerDisplayLoop extends Thread
    {
        public void run()
        {
            while (opModeIsActive())
            {
                telemetry.addData("1", "LFM: " + leftFrontMotor.getCurrentPosition());
                telemetry.addData("2", "RFM: " + rightFrontMotor.getCurrentPosition());
                telemetry.addData("3", "LBM: " + leftBackMotor.getCurrentPosition());
                telemetry.addData("4", "RBM: " + rightBackMotor.getCurrentPosition());

                telemetry.addData("5", "Red: " + colorSensor.red());
                telemetry.addData("6", "Green: " + colorSensor.green());
                telemetry.addData("7", "Blue: " + colorSensor.blue());

                /*
                telemetry.addData("5", "Swivel: " + swivelServo.getPosition());
                telemetry.addData("6", "Arm: " + armServo.getPosition());
                telemetry.addData("7", "Tilt: " + doorServo.getPosition());
                */
                telemetry.addData("8", "Time Elapsed:" + gameTimer.time());
            }
        }
    }

    public ProgramType getProgramType () //override in any meaningful subclass
    {
        return ProgramType.UNDECIDED;
    }

    public boolean runConditions()
    {
        int maxTime;

        if (getProgramType() == ProgramType.AUTONOMOUS)
        {
            maxTime = 120000 - TIMER_STOP_BUFFER;
        }

        else if (getProgramType() == ProgramType.TELEOP)
        {
            maxTime = 30000 - TIMER_STOP_BUFFER;
        }

        boolean timeValid = (!TIMER_ON || (gameTimer.time() < maxTime));
        return (opModeIsActive() && timeValid);

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

    public void sleep(int millis) //change back to old way if the new way doesn't work
    {
        /*
        try
        {
            Thread.sleep(millis);
        }

        catch (Exception e)
        {
            System.exit(0);
        }
        */

        int startTime = gameTimer.time();
        while (runConditions() && (gameTimer.time() < startTime + millis))
        {

        }
        return;
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
            //telemetry.addData("9", "" + leftFrontMotor.getCurrentPosition());
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
        double updateTime = ((mode == ENCODER) ? ENCODER_SYNC_UPDATE_TIME : GYRO_SYNC_UPDATE_TIME);

        resetDriveEncoders();
        setDrivePower(power);

        while (runConditions() && !driveEncodersHaveReached(encoderCount))
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
        while (runConditions() && !driveEncodersHaveReached(count))
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
        while (runConditions() && !turnEncodersHaveReached(distanceToEncoderCount(distance)))
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

    public static final boolean CLOSE = false;
    public static final boolean OPEN = true;

    public final void moveDoor (boolean position)
    {
        if (position == OPEN)
        {
            doorServo.setPosition(0.0);
        }

        else if (position == CLOSE)
        {
            doorServo.setPosition(1.0);
        }

    }

    public final void moveDoor()
    {
        doorServo.setPosition(doorServo.getPosition() == 0.0 ? 1.0 : 0.0); //set door to whatever position it wasn't in before.
    }
}