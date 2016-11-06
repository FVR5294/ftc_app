package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/***
 * robotconfig is a simple and effective way to import the robot configuration information into every program.
 * To enable it, simply add the code
 *
 * @code robotconfig robot = new robotconfig();
 * into the op mode file.
 * Every new motor and sensor should be added to this file.
 */
public class robotconfig {

    public static final int LED_CHANNEL = 5;
    static public DataLogger dl;
    private final double MAX_POWER = 1.00;
    private final double MIN_POWER = 0.05;
    private final double DEAD_ZONE = 0.02;
    public DcMotor fLeftMotor = null;
    public DcMotor fRightMotor = null;
    public DcMotor bLeftMotor = null;
    public DcMotor bRightMotor = null;
    public Servo buttonPusher = null;//now public, because of Ben
    public ColorSensor sensorRGB;
    public DeviceInterfaceModule cdim;
    public boolean bLedOn = false;
    public HardwareMap hwMap = null;
    public LinearOpMode linearOpMode;
    public OpMode opMode;
    public Telemetry ltelemetry;
    public boolean debugMode = true;


    /* Constructor */
    public robotconfig() {

    }

    static public void addlog(DataLogger dli, String function, String message) {
        dli.addField(function);
        dli.addField(message);
        dli.newLine();
    }

    /***
     * enables the motor break for the drive train motors.
     * This function only has to be run if one wants the break to be enabled.
     */
    public void enableMotorBreak() {
        addlog(dl, "robot", "enableMotorBreak was called");
        if (debugMode) {
            return;
        }
        fLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /***
     * disables the motor break for the drive train motors.
     * This function only has to be run if one wants disable the break after it is manually enabled
     */
    public void disableMotorBreak() {
        addlog(dl, "robot", "disableMotorBreak was called");
        if (debugMode) {
            return;
        }
        fLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        fRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        bLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        bRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    /***
     * resets the encoders of the drive train motors but doesn't put them back to the normal mode
     */
    public void resetMotorEncoders() {
        addlog(dl, "robot", "resetMotorEncoders was called");
        if (debugMode) {
            return;
        }
        fLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    /***
     * makes the drive train motors use the RUN_USING_ENCODER mode
     */
    public void enableMotorEncoders() {
        addlog(dl, "robot", "enableMotorEncoders was called");
        if (debugMode) {
            return;
        }
        fLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setMotorMaxSpeed() {
        fLeftMotor.setMaxSpeed(1000);
        fRightMotor.setMaxSpeed(1000);
        bLeftMotor.setMaxSpeed(1000);
        bRightMotor.setMaxSpeed(1000);
    }

    /***
     * makes the drive train motors use the RUN_TO_POSITION mode
     */
    public void enableEncodersToPosition() {
        addlog(dl, "robot", "enableEncodersToPosition was called");
        if (debugMode) {
            return;
        }
        fLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    /***
     * sets the drive train mobor mode to RUN_WITHOUT_ENCODER
     */
    public void disableMotorEncoders() {
        addlog(dl, "robot", "disableMotorEncoders was called");
        if (debugMode) {
            return;
        }
        fLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /***
     * sets the target position of all the motors in pulses. probably won't work as expected if less than 2 values are equal to zero
     *
     * @param forward pulses in the forward direction
     * @param right   pulses in sliding to the right (or left if negative)
     * @param spin    pulses in spinning clockwise
     */
    public void setMotorTargets(int forward, int right, int spin) {
        addlog(dl, "robot", "setMotorTargets was called - f:r:s: " + String.format("%d", forward) + " : " + String.format("%d", right) + " : " + String.format("%d", spin));
        if (debugMode) {
            return;
        }
        fLeftMotor.setTargetPosition(fLeftMotor.getCurrentPosition() + forward + right + spin);
        fRightMotor.setTargetPosition(fRightMotor.getCurrentPosition() + forward - right - spin);
        bLeftMotor.setTargetPosition(bLeftMotor.getCurrentPosition() + forward - right + spin);
        bRightMotor.setTargetPosition(bRightMotor.getCurrentPosition() + forward + right - spin);
    }

    /***
     * sets power of drive train motors to a value, for run to position mode
     *
     * @param power power for all motors
     */
    public void setMotorPower(double power) {
        addlog(dl, "robot", "setMotorPower was called - power: " + String.format("%.2f", power));
        if (debugMode) {
            return;
        }
        fLeftMotor.setPower(power);
        fRightMotor.setPower(power);
        bLeftMotor.setPower(power);
        bRightMotor.setPower(power);
    }

    /***
     * simply detects if one of the drive train motors is busy in run to position mode
     *
     * @return true if any one of the motors are busy
     */
    public boolean isMotorBusy() {
        addlog(dl, "robot", "isMotorBusy was called");
        if (debugMode) {
            return (false);
        }
        return fLeftMotor.isBusy() || fRightMotor.isBusy() || bLeftMotor.isBusy() || bRightMotor.isBusy();
    }

    /* Initialize standard Hardware interfaces - LinearOpMode */

    public void init(LinearOpMode linearOpMode) {

        this.linearOpMode = linearOpMode;

        // Save reference to Hardware map in class variable

        hwMap = linearOpMode.hardwareMap;

        dl = new DataLogger("10635", "autonomousTest");
        addlog(dl, "r.init", "r.init was invoked (a)");

        // Define and Initialize Motors
        if (!debugMode) {

            fLeftMotor = hwMap.dcMotor.get("fl_drive");
            fRightMotor = hwMap.dcMotor.get("fr_drive");
            bLeftMotor = hwMap.dcMotor.get("bl_drive");
            bRightMotor = hwMap.dcMotor.get("br_drive");

            setMotorMaxSpeed();

            //get sensor stuff
            cdim = hwMap.deviceInterfaceModule.get("dim");
            sensorRGB = hwMap.colorSensor.get("color");

            //initialize sensor stuff
            cdim.setDigitalChannelMode(LED_CHANNEL, DigitalChannelController.Mode.OUTPUT);
            cdim.setDigitalChannelState(LED_CHANNEL, bLedOn);

            // And initialize servo
            buttonPusher = hwMap.servo.get("buttonPusher");

            fLeftMotor.setDirection(DcMotor.Direction.FORWARD); //forward for andymark motor in drivetrain
            bLeftMotor.setDirection(DcMotor.Direction.FORWARD); //forward for andymark motor in drivetrain
            fRightMotor.setDirection(DcMotor.Direction.REVERSE); //reverse for andymark motor in drivetrain
            bRightMotor.setDirection(DcMotor.Direction.REVERSE); //reverse for andymark motor in drivetrain

            // Set all motors to zero power
            fLeftMotor.setPower(0);
            fRightMotor.setPower(0);
            bLeftMotor.setPower(0);
            bRightMotor.setPower(0);
        }
        //and check servo power
        this.pushButton(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        this.resetMotorEncoders();
        Thread.yield();
        this.setMotorTargets(0, 0, 0);
        Thread.yield();
        this.enableMotorEncoders();
        Thread.yield();
        this.disableMotorBreak();
        Thread.yield();

        addlog(dl, "r.init", "r.init finished (a)");
    }

        /* Initialize standard Hardware interfaces */

    public void init(OpMode opMode) {
        this.opMode = opMode;

        // Save reference to Hardware map in class variable

        hwMap = opMode.hardwareMap;

        dl = new DataLogger("10635", "TeleOp");
        addlog(dl, "r.init", "r.init was invoked (t)");

        // Define and Initialize Motors
        fLeftMotor = hwMap.dcMotor.get("fl_drive");
        fRightMotor = hwMap.dcMotor.get("fr_drive");
        bLeftMotor = hwMap.dcMotor.get("bl_drive");
        bRightMotor = hwMap.dcMotor.get("br_drive");

        //get sensor stuff
        cdim = hwMap.deviceInterfaceModule.get("dim");
        sensorRGB = hwMap.colorSensor.get("color");

        //initialize sensor stuff
        cdim.setDigitalChannelMode(LED_CHANNEL, DigitalChannelController.Mode.OUTPUT);
        cdim.setDigitalChannelState(LED_CHANNEL, bLedOn);

        // And initialize servo
        buttonPusher = hwMap.servo.get("buttonPusher");

        fLeftMotor.setDirection(DcMotor.Direction.FORWARD); //forward for andymark motor in drivetrain
        bLeftMotor.setDirection(DcMotor.Direction.FORWARD); //forward for andymark motor in drivetrain
        fRightMotor.setDirection(DcMotor.Direction.REVERSE); //reverse for andymark motor in drivetrain
        bRightMotor.setDirection(DcMotor.Direction.REVERSE); //reverse for andymark motor in drivetrain

        // Set all motors to zero power
        fLeftMotor.setPower(0);
        fRightMotor.setPower(0);
        bLeftMotor.setPower(0);
        bRightMotor.setPower(0);

        //and check servo power
        this.pushButton(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        this.resetMotorEncoders();
        Thread.yield();
        this.setMotorTargets(0, 0, 0);
        Thread.yield();
        this.enableMotorEncoders();
        Thread.yield();
        this.disableMotorBreak();
        Thread.yield();

        addlog(dl, "r.init", "r.init finished (t)");
    }

    /***
     * pushButton is a function to move the button pusher servo to press either one of the buttons, or reset its position.
     *
     * @param button input 1 for the left button, 2 for the right button, and any other number to reset it back to center
     */
    public void pushButton(int button) {
        addlog(dl, "robot", "pushButton was invoked");
        switch (button) {
            case 1:
                buttonPusher.setPosition(0.3);//right button
                break;
            case -1:
                buttonPusher.setPosition(0.7);//left button
                break;
            default:
                buttonPusher.setPosition(0.5);
                break;
        }//*/
    }

    /***
     * code that will one day detect the color of the beacon
     *
     * @return 1 for red, -1 for blue, 0 if can't tell
     */
    public int detectColor() {

        addlog(dl, "robot", "detectColor was called");

        if (debugMode) {
            return (1);  // if no hardware, let's assume red
        }
        if (sensorRGB.red() > sensorRGB.blue()) {
            return 1;
        } else if (sensorRGB.blue() > sensorRGB.red()) {
            return -1;
        } else {
            return 0;
        }
    }

    /***
     * move is a function to efficiently set the power values of all 4 drive train motors in one quick line.
     *
     * @param forward double: ranges from 1=forward to -1=backward
     * @param right   double: ranges from 1=slide right to -1=slide left
     * @param spin    double: ranges from 1=turn clockwise to -1=turn counterclockwise
     */
    public void move(double forward, double right, double spin) {

        addlog(dl, "robot", "move was called - f:r:s: " + String.format("%.2f", forward) + " : " + String.format("%.2f", right) + " : " + String.format("%.2f", spin));

        if (debugMode) {
            return;
        }

        fLeftMotor.setPower(scale(forward) + scale(right) + scale(spin));
        fRightMotor.setPower(scale(forward) - scale(right) - scale(spin));
        bLeftMotor.setPower(scale(forward) - scale(right) + scale(spin));
        bRightMotor.setPower(scale(forward) + scale(right) - scale(spin));

    }

    /***
     * function to scale the output of the joystick
     *
     * @param input unscaled joystick values
     * @return scaled joystick values
     */
    public double scale(double input) {

        double sign = 1.0;
        double output;

        if (input < 0.0) {
            sign = -1.0;    // remember incoming joystick direction, -1 is fully up
        }

        input = input * input;  // power transfer curve, adjust sign handling if sign is preserved by this function

        if (input < (DEAD_ZONE * DEAD_ZONE)) { // need to square DEAD_ZONE since input is already squared
            return (0);
        }

        output = (input - (DEAD_ZONE * DEAD_ZONE));             // shift so that range is from 0 to 1.0-DEAD_ZONE^2 instead of DEAD_ZONE^2 to 1.0
        output = output / (1.0 - (DEAD_ZONE * DEAD_ZONE));     // scale so 0 to 1.0-DEAD_ZONE^2 is now 0 to 1
        output = output * (MAX_POWER - MIN_POWER);              // scale so range is now 0 to (MAX-MIN)
        output = output + MIN_POWER;                            // shift so range is now MIN_POWER to MAX_POWER
        return (output * sign);         // don't forget to restore the sign of the input
    }

}

