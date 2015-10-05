package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.*;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.*;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

/*
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
*/

//------------------------------------------------------------------------------
//
// PushBotHardware
//
//--------
// Extends the OpMode class to provide a single hardware access point for the
// Push Bot.
//--------
public class Old_5220_OpMode extends OpMode
{
    //--------------------------------------------------------------------------
    //
    // driveController
    //
    //--------
    // This class member manages the aspects of the left drive motor.
    //--------

    protected DcMotorController driveController; //MAKE SURE THESE THINGS HAVE SAME NAME AS IN PHONE CONFIGURATION

    protected DcMotor leftMotor;
    final int leftMotorChannel = 1;
    /*
        private DcMotor rightMotor;
        final int rightMotorChannel = 2;
    */
    Servo servoL;
    Servo servoR;

    protected int phase;

    //--------
    public Old_5220_OpMode()
    {



        super();
        //
        // Initialize base classes.
        //
        // All via self-construction.

        //
        // Initialize class members.
        //
        // All via self-construction.

    } // PushBotManual::PushBotHardware

    public class Stopwatch {

        private final long start;

        /**
         * Initializes a new stopwatch.
         */
        public Stopwatch() {
            start = System.currentTimeMillis();
        }


        /**
         * Returns the elapsed CPU time (in seconds) since the stopwatch was created.
         *
         * @return elapsed CPU time (in seconds) since the stopwatch was created
         */
        public double elapsedTime() {
            long now = System.currentTimeMillis();
            return (now - start) / 1000.0;
        }

    }

    public void setDrivePower (int power)
    {

    }


    //--------------------------------------------------------------------------
    //
    // init
    //
    //--------
    // Performs any actions that are necessary when the OpMode is enabled.
    //
    // The system calls this member once when the OpMode is enabled.

    public void sleep (int millis)
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

    public void phaseSwitch () //override me in extending class, whatever the body is here is just for early testing
    {

    }

    public void update_telemetry ()

    {
        //
        // Send telemetry data to the driver station.
        //
        telemetry.addData ("01", "Hello world!");
        telemetry.addData ("02", "servoL: " + servoL.getPosition());
        telemetry.addData ("03", "servoR: " + servoR.getPosition());

        //--------
    }
    @Override public void init ()
    {
        driveController = hardwareMap.dcMotorController.get("motorController_P0");

        leftMotor = hardwareMap.dcMotor.get ("motor_P0_1");
        driveController.setMotorChannelMode(1, DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        driveController.setMotorChannelMode(2, DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);

        //rightMotor = hardwareMap.dcMotor.get ("rightMotor");
        // rightMotor.setDirection (DcMotor.Direction.REVERSE);

        double initServoPosition = 0.5;

        double l_hand_position = 0.5;

        servoL = hardwareMap.servo.get ("servo_P1_1");
        servoL.setPosition (l_hand_position);

        servoR = hardwareMap.servo.get ("servo_P1_2");
        //servoR.setPosition(l_hand_position);





    }

    //--------------------------------------------------------------------------
    //
    // start
    //
    //--------
    // Performs any actions that are necessary when the OpMode is enabled.
    //
    // The system calls this member once when the OpMode is enabled.
    //--------
    @Override public void start ()

    {
        update_telemetry();
        leftMotor.setPower(0.8); //PURELY A TEST, DELETE LATER
        driveController.setMotorPower(1, 0.8);
        driveController.setMotorPower(2, 0.8);
        sleep (2000);
        leftMotor.setPower(0);
        sleep (1000);
        servoL.setPosition(0.7);
        //servoR.setPosition (0.6);

        //
        // Only actions that are common to all Op-Modes (i.e. both automatic and
        // manual) should be implemented here.

        //PUT EVERYTHING HERE IN AUTONOMOUS PROGRAM AND THEN KILL PROGRAM BEFORE LOOP

    } // PushBotHardware::start

    //--------------------------------------------------------------------------
    //
    // loop
    //
    //--------
    // Performs any actions that are necessary while the OpMode is running.
    //
    // The system calls this member repeatedly while the OpMode is running.
    //--------
    @Override public void loop ()

    {
        //
        // Only actions that are common to all OpModes (i.e. both auto and\
        // manual) should be implemented here.
        //
        // This method is designed to be overridden.
        //
        //ADD THINGS LIKE CONSTANT GYRO SENSOR MONITORING HERE< IF NOT IN SEPERATE THREAD

        update_telemetry (); // Update common telemetry
        phaseSwitch();

    } // PushBotHardware::loop


    //--------------------------------------------------------------------------
    //
    // stop
    //
    //--------
    // Performs any actions that are necessary when the OpMode is disabled.
    //
    // The system calls this member once when the OpMode is disabled.
    //--------
    @Override public void stop ()
    {
        //
        // Nothing needs to be done for this OpMode.
        //

    } // PushBotHardware::stop

    //HELPER METHODS:

    double scale_motor_power (double p_power)
    {
        //
        // Assume no scaling.
        //
        double l_scale = 0.0f;

        //
        // Ensure the values are legal.
        //
        double l_power = Range.clip (p_power, -1, 1);

        double[] l_array =
                { 0.00, 0.05, 0.09, 0.10, 0.12
                        , 0.15, 0.18, 0.24, 0.30, 0.36
                        , 0.43, 0.50, 0.60, 0.72, 0.85
                        , 1.00, 1.00
                };

        //
        // Get the corresponding index for the specified argument/parameter.
        //
        int l_index = (int) (l_power * 16.0);
        if (l_index < 0)
        {
            l_index = -l_index;
        }
        else if (l_index > 16)
        {
            l_index = 16;
        }

        if (l_power < 0)
        {
            l_scale = -l_array[l_index];
        }
        else
        {
            l_scale = l_array[l_index];
        }

        return l_scale;

    }

    void setDrivePower (double lp, double rp)
    {
        leftMotor.setPower (lp);
        //rightMotor.setPower (rp);

    }

    void setDrivePower (double p)
    {
        setDrivePower(p, p);
    }

    //--------------------------------------------------------------------------
    //
    // run_using_encoders
    //
    /**
     * Sets both drive wheel encoders to run, if the mode is appropriate.
     */
    /*
    public void run_using_encoders ()

    {
        DcMotorController.RunMode l_mode = driveController.getMotorChannelMode ( leftMotorChannel);

        if (l_mode == DcMotorController.RunMode.RESET_ENCODERS)
        {
            driveController.setMotorChannelMode(leftMotorChannel, DcMotorController.RunMode.RUN_USING_ENCODERS);
        }

        l_mode = driveController.getMotorChannelMode( rightMotorChannel);
        if (l_mode == DcMotorController.RunMode.RESET_ENCODERS)
        {
            driveController.setMotorChannelMode( rightMotorChannel, DcMotorController.RunMode.RUN_USING_ENCODERS);
        }

    }
*/
    //--------------------------------------------------------------------------
    //
    // reset_drive_encoders
    //
    /**
     * Resets both drive wheel encoders.
     */
    public void reset_drive_encoders ()

    {
        //
        // Reset the motor encoders on the drive wheels.
        //
        driveController.setMotorChannelMode( leftMotorChannel, DcMotorController.RunMode.RESET_ENCODERS);
        // driveController.setMotorChannelMode( rightMotorChannel, DcMotorController.RunMode.RESET_ENCODERS);

    } // PushBotAuto::reset_drive_encoders

    //NOTE: encoder value is, for example, leftMotor.getCurrentPosition();

    boolean have_drive_encoders_reached( double p_left_count, double p_right_count)
    {
        //
        // Assume failure.
        //
        boolean l_status = false;

        //
        // Have the encoders reached the specified values?
        //
        // TODO Implement stall code using these variables.
        //
        if ((Math.abs (leftMotor.getCurrentPosition ()) > p_left_count) /*&& (Math.abs (rightMotor.getCurrentPosition ()) > p_right_count)*/)
        {
            l_status = true;
        }

        return l_status;

    }

    //--------------------------------------------------------------------------
    //
    // have_drive_encoders_reset
    //
    //--------
    // Scale the joystick input using a nonlinear algorithm.
    //--------
    boolean resetDriveEncoders ()
    {
        //
        // Assume failure.
        //

        boolean l_status = false;

        //
        // Have the encoders reached zero?
        //
        if ((leftMotor.getCurrentPosition() == 0) /*&& (rightMotor.getCurrentPosition () == 0)*/)
        {
            //
            // Set the status to a positive indication.
            //
            l_status = true;
        }

        //
        // Return the status.
        //
        return l_status;

    } // PushBotManual::have_drive_encoders_reset

    void setServoPositions (double position)
    {
        double l_position = Range.clip( position, Servo.MIN_POSITION, Servo.MAX_POSITION
        );

        //
        // Set the value.  The right hand value must be opposite of the left
        // value.
        //
        servoL.setPosition (l_position);
        //servoR.setPosition (1.0 - l_position);

    }
}
