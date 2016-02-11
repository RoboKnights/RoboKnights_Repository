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

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.ftcrobotcontroller.FtcRobotControllerActivity;
import com.qualcomm.ftcrobotcontroller.R;
import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.util.*;
//hello!

//Currently using FTC SDK released 11-4-2015

//add program off switch for using any one particular motor

/* TODO (in priority order from highest to lowest):


Fine tune autonomous program

Get the TeleOp direction switch button and slow driving to work

DEFINITELY add joystick control to autonomous config loop (decide program priority between that and touch sensors later). Simplest way will be using left and right bumpers. Other ways are possible.

Add a way to display autonomous config options on the robot controller phone?

 */

public abstract class OpMode_5220 extends LinearOpMode //FIGURE OUT HOW TO GET DECENT VERSION CONTROL INSTEAD OF HAVING TO SAVE AS FOR EVERY NEW VERSION
{
    //CONSTANTS:

    protected static final int HAS_NOT_STARTED = 0;
    protected static final int SETUP = 1;
    protected static final int INIT = 2;
    protected static final int WAITING = 3;
    protected static final int RUNNING = 4;

    protected static final boolean BLUE = true;
    protected static final boolean RED = false;
    protected static final boolean RIGHT = true;
    protected static final boolean LEFT = false;

    protected static enum ProgramType {UNDECIDED, AUTONOMOUS, TELEOP};
    protected static ProgramType programType = ProgramType.UNDECIDED;

    protected static final int TELEOP_TIME_LIMIT = 1200000; //currently 20 minutes, more than enough for any single run.
    protected static final int AUTONOMOUS_TIME_LIMIT = 29000;

    protected static final double NORMAL = 2;
    protected static final double ENCODER = 3;
    protected static final double GYRO = 4;

    protected static final double WHEEL_DIAMETER = 6.0; //in inches
    protected static final double GEAR_RATIO = 1.0 / 2.0;
    protected static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;
    protected static final int ENCODER_COUNTS_PER_ROTATION = 1440;

    protected static final double SWIVEL_INIT = 0.7529; //// may be reset in TeleOp
    protected static final double SWIVEL_180 = 0.23;
    protected static final double SWIVEL_360 = SWIVEL_180 * 2;

    //CONFIGURABLE CONSTANTS:

    protected static final boolean TIMER_ON = false;
    protected static final int TIMER_STOP_BUFFER = 500; //in millis

    protected static final double DEFAULT_DRIVE_POWER = 1.0;
    protected static final double DEFAULT_SYNC_POWER = 0.56;
    protected static final double DEFAULT_TURN_POWER = 0.30;
    protected static final double DEFAULT_TURN_POWER_HIGH = 0.80;
    protected static final double INIT_SERVO_POSITION = 0.5;

    protected static final double ENCODER_SYNC_PROPORTIONALITY_CONSTANT = 0.001; //0.001 means 50 encoder counts --> 5% power difference
    protected static final double GYRO_SYNC_PROPORTIONALITY_CONSTANT = 0.02; //this times 100 is the motor power difference per degree off.
    protected static final double ENCODER_SYNC_UPDATE_TIME = 20; //in milliseconds for convenience
    protected static final double GYRO_SYNC_UPDATE_TIME = 20; //in milliseconds for convenience

    protected static final double CLIMBER_FLING_TIME = 1.0;

    protected static final double LINE_WHITE_THRESHOLD = 50;

    //MOTORS AND SERVOS:

    protected static final String[] motorNames = {}; //Fill this in later.

    protected DcMotor leftFrontMotor;
    protected DcMotor rightFrontMotor;
    protected DcMotor leftBackMotor;
    protected DcMotor rightBackMotor;
    //protected DcMotor pullMotor1;
   // protected DcMotor pullMotor2;
    protected DcMotor sweeperMotor1;
    protected DcMotor sweeperMotor2;
    protected DcMotor slideMotor;

    protected DcMotor[] driveMotors = new DcMotor[4];
    protected int[] driveMotorInitValues = new int[4];

    protected Servo swivelServo;
    protected Servo releaseServo;
    protected Servo buttonServo;
    protected Servo leftWallServo;
    protected Servo rightWallServo;
    protected Servo leftDumpServo;
    protected Servo rightDumpServo;
    protected Servo leftClimberServo;
    protected Servo rightClimberServo;

    protected double swivelServoInit;

    //SENSORS:
    protected ColorSensor colorSensorFront;
    protected ColorSensor colorSensorDown;
    protected GyroSensor gyroSensor;
    protected TouchSensor touchSensor1;
    protected TouchSensor touchSensor2;
    protected TouchSensor touchSensorFront;

    //OTHER GLOBAL VARIABLES:

    protected FtcRobotControllerActivity ftcRCA;
    protected boolean programFinished = false; //allows manual termination of the program in an orderly fashion, especially for autonomous
    protected Stopwatch gameTimer;
    protected boolean isArmMoving = false;
    protected int phase = HAS_NOT_STARTED;
    protected double swivelPosition;

    public void setup()//this and the declarations above are the equivalent of the pragmas in RobotC
    {
        phase = SETUP;

        ftcRCA = FtcRobotControllerActivity.ftcRCA;

        hardwareMap.logDevices();

        leftFrontMotor = hardwareMap.dcMotor.get("lf");
        rightFrontMotor = hardwareMap.dcMotor.get("rf");
        leftBackMotor = hardwareMap.dcMotor.get("lb");
        rightBackMotor = hardwareMap.dcMotor.get("rb");

        rightFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        rightBackMotor.setDirection(DcMotor.Direction.REVERSE);
        //leftMidMotor.setDirection(DcMotor.Direction.REVERSE);

        driveMotors[0] = leftFrontMotor;
        driveMotors[1] = rightFrontMotor;
        driveMotors[2] = leftBackMotor;
        driveMotors[3] = rightBackMotor;

        for (DcMotor dcm: driveMotors) dcm.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
        for (DcMotor dcm: driveMotors) dcm.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);

        sweeperMotor1 = hardwareMap.dcMotor.get("sweeper1");
        sweeperMotor2 = hardwareMap.dcMotor.get("sweeper2");
        slideMotor = hardwareMap.dcMotor.get("slides");



        swivelServo = hardwareMap.servo.get("sServo");
        releaseServo = hardwareMap.servo.get("rServo");
        buttonServo = hardwareMap.servo.get("bServo");
        leftWallServo = hardwareMap.servo.get ("lwServo");
        rightWallServo = hardwareMap.servo.get ("rwServo");
        leftDumpServo = hardwareMap.servo.get("ldServo");
        rightDumpServo = hardwareMap.servo.get("rdServo");
        leftClimberServo = hardwareMap.servo.get("lcServo");
        rightClimberServo = hardwareMap.servo.get("rcServo");

        colorSensorDown = hardwareMap.colorSensor.get("cSensor1");
        colorSensorFront = hardwareMap.colorSensor.get("cSensor2");
        colorSensorFront.setI2cAddress(0x3E); //in hex, 0x3e = 62. deault address is 60 (reserved for colorSensorDown)
        colorSensorFront.enableLed(false); //make sure this method works as it's supposed to
        colorSensorDown.enableLed(false); //make sure this method works as it's supposed to
        gyroSensor = hardwareMap.gyroSensor.get("gSensor");
        touchSensorFront = hardwareMap.touchSensor.get("tSensor1");
    }

    public void initialize()
    {
        //swivelServoInit = swivelServo.getPosition();
       // setCustomSkin();
        moveDumper(DOWN);
        leftClimberServo.setPosition(0.5);
        rightClimberServo.setPosition(0.5);
        buttonServo.setPosition(0.1);
        swivelServo.setPosition(SWIVEL_INIT);

        gyroSensor.calibrate();
        gyroSensor.resetZAxisIntegrator();

        moveWall(DOWN);
        phase = INIT;

        writeToLog ("Down: " + colorSensorDown.getI2cAddress());
        writeToLog ("Front: " + colorSensorFront.getI2cAddress());
    }

    public void waitForStart () throws InterruptedException
    {
        phase = WAITING;
        super.waitForStart();
    }

    public abstract void main(); //implement in all subclasses. This is the main body of the program. Maybe also make initializeRobot something to override if its different between OpModes.

    public final void runOpMode() throws InterruptedException
    {
        setup();
        initialize();
        waitForStart();

        phase = RUNNING;
        gameTimer = new Stopwatch();
        releasePin();

        main();
        end();
    }

    public void end()
    {
       stopDrivetrain();
    }

    //HELPER CLASSES AND METHODS:
    //______________________________________________________________________________________________________________
   /* public class DcMotor5220
    {
        private DcMotor dcMotor;
        private int encoderInit = 0;

        public DcMotor5220 (DcMotor dcMotor)
        {

        }

        public void resetEncoder ()
        {
            encoderInit = getCurrentPosition();
        }

        public int getCurrentPosition ()
        {
            return super.getCurrentPosition() - encoderInit;
        }

        public int getAbsoluteCurrentPosition ()
        {
            return super.getCurrentPosition();
        }
    }
*/
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
                telemetry.addData("1", "LFM: " + leftFrontMotor.getCurrentPosition() + ", RFM: " + rightFrontMotor.getCurrentPosition());
                telemetry.addData("2", "LBM: " + leftBackMotor.getCurrentPosition() + ", RBM: " + rightBackMotor.getCurrentPosition());
                telemetry.addData("3", "Swivel: " + swivelServo.getPosition());
                telemetry.addData("4", "Dumper: " + leftDumpServo.getPosition());

                //telemetry.addData("5", "Front: R = " + colorSensorFront.red() + ", G = " + colorSensorFront.green() + ", B = " + colorSensorFront.blue());
                telemetry.addData("5", "Down: R = " + colorSensorDown.red() + ", G = " + colorSensorDown.green() + ", B = " + colorSensorDown.blue() + ", A = " +  colorSensorDown.alpha());
                telemetry.addData("6", "Front: R = " + colorSensorFront.red() + ", G = " + colorSensorFront.green() + ", B = " + colorSensorFront.blue() + ", A = " +  colorSensorFront.alpha());
                telemetry.addData("7", "Gyro H: " + getGyroDirection() /*+ ", Front Touch: " + touchSensorFront.isPressed()*/);
                //telemetry.addData("7", "Touch: " + touchSensor1.isPressed());

                //telemetry.addData("7", "Beacon: " + getRescueBeaconColor());

                telemetry.addData("8", "Time Elapsed:" + gameTimer.time());
                //waitOneFullHardwareCycle();
            }
        }
    }

    public void writeToLog (String toWrite)
    {
        if (!runConditions()) return;
        String text = "USER MESSAGE: ";
        for (int i = 0; i < 54; i++) text = text + "*";
        for (int i = 0; i < 54; i++) text = text + " ";
        text = text + " " + toWrite + " ";
        for (int i = 0; i < 54; i++) text = text + " ";
        for (int i = 0; i < 54; i++) text = text + "_";
        //DbgLog.error(text);
        DbgLog.msg(text);
        DbgLog.msg("USER MESSAGE (short): " + toWrite);
        DbgLog.error("USER MESSAGE (short): " + toWrite);
    }

    public void setCustomSkin() //maybe transfer this sort of app modification stuff to ftcrobotcontrolleractivity. It might be more appropriate there.
    {
        View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(R.id.scanButton);
        relativeLayout.setBackgroundColor(Color.CYAN);
        View button1 = ((Activity) hardwareMap.appContext).findViewById(R.id.file_activate_button); //try different button if this doesn't work.
        View.OnLongClickListener listener = new View.OnLongClickListener()
        {
            public int numPresses = 0;
            public boolean onLongClick(View v)
            {
                telemetry.addData("4", "Button clicks: " + numPresses++);
                return false;
            }
        };
        button1.setOnLongClickListener(listener);
        View entireScreen = ((Activity) hardwareMap.appContext).findViewById(R.id.entire_screen); //try different button if this doesn't work.
        View button2 = ((Activity) hardwareMap.appContext).findViewById(R.id.textWifiDirectStatus); //try different button if this doesn't work.
        //button2.setOnLongClickListener(); //maybe put new teleOp thing here.
        //try adding a new listener to the buttons to make them do different things.
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
            //maxTime = 120000 - TIMER_STOP_BUFFER;
            maxTime = AUTONOMOUS_TIME_LIMIT;
        }

        else if (getProgramType() == ProgramType.TELEOP)
        {
            return (opModeIsActive());
            //maxTime = 30000 - TIMER_STOP_BUFFER;
        }

        boolean timeValid = (!TIMER_ON || (gameTimer.time() < maxTime));
        return (opModeIsActive() && (!programFinished) && timeValid);
    }

    public int distanceToEncoderCount (double distance) //distance is in inches
    {
        double wheelRotations = distance / WHEEL_CIRCUMFERENCE;
        double motorRotations = wheelRotations / GEAR_RATIO;
        long encoderCounts = Math.round(motorRotations * ENCODER_COUNTS_PER_ROTATION);
        return (int) encoderCounts;
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
            //waitFullCycle();
        }
        return;
    }

    public final void waitFullCycle ()
    {
        if (!runConditions()) return; //NOT SURE IF PUTTING THIS HERE IS A GOOD IDEA, TEST IT TO SEE IF IT IS OK.

        try
        {
            waitOneFullHardwareCycle();
        }
        catch (Exception e)
        {
            e.printStackTrace();
        }
    }

    public final void waitNextCycle ()
    {
        try
        {
            waitForNextHardwareCycle();
        }
        catch (Exception e)
        {
            e.printStackTrace();
        }
    }

    public double getGyroDirection () //placeholder
    {
        //return gyroSensor.getRotation();
        return gyroSensor.getHeading();
        //return 42.0; //testing
    }

    public void restartRobot()
    {
        ftcRCA.requestRobotRestart();
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

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
        //setMotorPower(leftMidMotor, power);
    }

    public final void setRightDrivePower (double power)
    {
        setMotorPower (rightFrontMotor, power);
        setMotorPower (rightBackMotor, power);
       // setMotorPower(rightMidMotor, power);
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
    {/*
        for (DcMotor motor: driveMotors)
        {
            motor.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
            motor.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        }
        */

        for (DcMotor dcm: driveMotors)
        {
            resetEncoder(dcm);
        }

    }

    public int getEncoderValue (DcMotor dcm)
    {
        return (dcm.getCurrentPosition() - driveMotorInitValues[motorToNumber(dcm)]);
    }

    public int motorToNumber (DcMotor dcm)
    {
        if (dcm == leftFrontMotor)
        {
            return 0;
        }

        if (dcm == rightFrontMotor)
        {
            return 1;
        }

        if (dcm == leftBackMotor)
        {
            return 2;
        }

        if (dcm == rightBackMotor)
        {
            return 3;
        }

        else
        {
            return -1;
        }
    }

    public void resetEncoder (DcMotor dcm)
    {
        driveMotorInitValues[motorToNumber(dcm)] = dcm.getCurrentPosition();
    }

    public boolean hasEncoderReached (DcMotor dcm, int encoderCount) //assumes that encoders start at 0 and are not moving to zero.
    {
        if (motorToNumber(dcm) == -1) return false;

        if (encoderCount > 0)
        {
            if (getEncoderValue(dcm) < encoderCount) return false;
            else return true;
        }

        else if (encoderCount < 0)
        {
            if (getEncoderValue(dcm) > encoderCount) return false;
            else return true;
        }

        else //encoderCount is 0
        {
            return (getEncoderValue(dcm) == 0);
        }
    }

    public final boolean driveEncodersHaveReached(int encoderCount) //need to modify this, or just eliminate it and put the condidition stuff in the main move method.
    {
        return (hasEncoderReached(leftFrontMotor, encoderCount) && hasEncoderReached(rightFrontMotor, encoderCount));
    }


    public final boolean turnEncodersHaveReached(int encoderCount)
    {
        return (hasEncoderReached(leftFrontMotor, encoderCount) && hasEncoderReached(rightFrontMotor, -encoderCount)); //make sure the minus sign on rightFrontMotor works.
    }

    public String getModeText (double mode)
    {
        if (mode == NORMAL) return "Normal";
        else if (mode == ENCODER) return "Encoder";
        else if (mode == GYRO) return "Gyro";
        else return "";
    }

    public final void move (double distance, double... params)
    {
        if (!runConditions()) return;

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

        //Main method body:

        if (power * distance < 0)
        {
            power = -power;
        }

        int encoderCount = distanceToEncoderCount(distance);
        writeToLog("MOVING: Distance = " + distance + ", Encoder Count = " + encoderCount + ", Mode = " + getModeText(mode) + ", Power = " + power);
        writeToLog("MOVING: UnReset encoder values are LFM: " + getEncoderValue(leftFrontMotor) + ", " + getEncoderValue(rightFrontMotor));
        double initialDirection = getGyroDirection();

        double powerChange = 0;
        double updateTime = ((mode == ENCODER) ? ENCODER_SYNC_UPDATE_TIME : GYRO_SYNC_UPDATE_TIME);

        resetDriveEncoders();
        writeToLog("MOVING: Initialized encoder values (should be 0) are LFM: " + getEncoderValue(leftFrontMotor) + ", RFM = " + getEncoderValue(rightFrontMotor));
        setDrivePower(power);

        while (runConditions() && !driveEncodersHaveReached(encoderCount)) //change back to runConditions if it works, change back to driveEncodersHaveReached if it works
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

            //waitFullCycle();

            //do nothing if mode is NORMAL.
        }
        stopDrivetrain();
        if (!runConditions()) return;
        //writeToLog("MOVING: Final encoder values are LFM: " + getEncoderValue(leftFrontMotor) + ", " + getEncoderValue(rightFrontMotor));
        waitFullCycle();
        waitFullCycle();
        sleep(99); //maybe reduce this if it wastes too much time to have this safety interval.
    }

    public void moveSimple (int count)
    {
        setDrivePower(DEFAULT_DRIVE_POWER);
        while (runConditions() && !driveEncodersHaveReached(count))
        {
            //waitFullCycle();
        }
        stopDrivetrain();
        waitFullCycle();
    }

    public final void moveTime(int time, double power)
    {
        writeToLog("Moving at " + power + " power for " + time + " ms");
        setDrivePower(power);
        sleep(time);
        stopDrivetrain();
        if (!runConditions()) return;
        waitFullCycle();
        stopDrivetrain();
    }

    //ROTATION:

    public final void setTurnPower (double power) //problm with this?
    {
        setLeftDrivePower(power);
        setRightDrivePower(-power);
    }

    public final void waitForGyroRotation (double degrees) //degrees must be less than 355
    {
        //convert degrees to proper value for this method
        gyroSensor.calibrate();
        while (gyroSensor.isCalibrating()) {};
        gyroSensor.resetZAxisIntegrator();
        sleep(500);
        if (degrees < 0)
        {
            while (runConditions() && (getGyroDirection() > (360 - degrees) || getGyroDirection() < 5))
            {

            }
        }

        else
        {
            while (runConditions() && (getGyroDirection() < degrees || getGyroDirection() > 355))
            {

            }
        }
    }

    public final void waitForAbsoluteGyroRotation (double degrees)
    {
        //finish later
    }

    public final void swingTurn (double distance, boolean side, double power)
    {
        if (power * distance < 0)
        {
            power = -power;
        }

        resetDriveEncoders();
        if (side == RIGHT) setRightDrivePower(power);
        else setLeftDrivePower(power);

        int encoderCount = distanceToEncoderCount(distance);

        while (runConditions() && (side == RIGHT ? !hasEncoderReached(rightFrontMotor, encoderCount) : !hasEncoderReached(leftFrontMotor, encoderCount))) //change back to runConditions if neecessary
        {

        }
        stopDrivetrain();
    }

    public final void swingTurn (double distance, boolean side)
    {

    }

    public final void rotate (double degrees, double power) //gyro rotation, add thing to make negative degrees = negative power.
    {
        if (power * degrees < 0) power = -power;
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
        if (power * distance < 0)
        {
            power = -power;
        }

        resetDriveEncoders();
        setTurnPower(power);
        while (runConditions() && !turnEncodersHaveReached(distanceToEncoderCount(distance))) //change back to runConditions if neecessary
        {
            //waitFullCycle();
        }
        stopDrivetrain();
        waitFullCycle();
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

    public static final double UP = 1.0;
    public static final double DOWN = 0.231;
    public static final double dumperHeight = 0.47;

    public final void moveDumper (double d)
    {
        leftDumpServo.setPosition(d);
        rightDumpServo.setPosition(1.0 - d);
    }

    public static final double COLLECT = SWIVEL_INIT;
    public static final double BLUE_HIGH = 0.863;
    public static final double BLUE_MEDIUM = 0.882;
    public static final double RED_HIGH = 0.66666;
    public static final double RED_MEDIUM = 0.635;

    public final void moveSwivel (double position)
    {
        swivelServo.setPosition(position);
    }

    public static final double leftWallInit = 0.48;
    public static final double rightWallInit = 0.5;
    public static final double wallOffset = 0.335;

    public final void moveWall (double position)
    {
        if (position == UP)
        {
            leftWallServo.setPosition(leftWallInit + wallOffset);
            rightWallServo.setPosition(rightWallInit - wallOffset);
        }

        else if (position == DOWN)
        {
            leftWallServo.setPosition(leftWallInit);
            rightWallServo.setPosition(rightWallInit);
        }
    }
    
    public final void setSweeperPower (double power)
    {
        sweeperMotor1.setPower(power);
        sweeperMotor2.setPower(power);
    }

    public final void releasePin()
    {
        releaseServo.setPosition(1.0);
    }

    public void flingClimbers ()
    {
        if (!runConditions()) return;
        sleep (200);
       // writeToLog ("Flinging Climbers.");
        Stopwatch climberTimer = new Stopwatch();
        double timeInMillis = CLIMBER_FLING_TIME * 1000.0;
        while (runConditions() && climberTimer.time() < timeInMillis)
        {
            double newPosition = (climberTimer.time() / timeInMillis);
            if (newPosition > 1) newPosition = 1;
            moveDumper (newPosition);
            //waitFullCycle(); //not sure if needed here
        }
        sleep (2000);
        moveDumper(DOWN);
       // writeToLog("Climber flinging is done.");

    }

    public Boolean getRescueBeaconColor () //experimental for the time being
    {
        double red = colorSensorFront.red();
        double blue = colorSensorFront.blue();
        double green = colorSensorFront.green();

        if (blue >= 1) return BLUE;
        else return RED;
    }

    public double getFloorBrightness ()
    {
        return (colorSensorDown.red() + colorSensorDown.green() + colorSensorDown.blue());
    }

}