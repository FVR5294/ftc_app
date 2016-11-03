package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

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
    public DcMotor fLeftMotor = null;
    public DcMotor fRightMotor = null;
    public DcMotor bLeftMotor = null;
    public DcMotor bRightMotor = null;
    public Servo buttonPusher = null;//now public, because of Ben
    public ColorSensor sensorRGB;
    public DeviceInterfaceModule cdim;
    public boolean bLedOn = false;
    HardwareMap hwMap = null;

    /* Constructor */
    public robotconfig() {

    }

    /***
     * enables the motor break for the drive train motors.
     * This function only has to be run if one wants the break to be enabled.
     */
    public void enableMotorBreak() {
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
        fLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        fRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        bLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        bRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    /***
     * resets the encoders of the drive train motors but doesn't put them back to the normal mode
     */
    public void resetMotorEncoders() {
        fLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    /***
     * makes the drive train motors use the RUN_USING_ENCODER mode
     */
    public void enableMotorEncoders() {
        fLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /***
     * makes the drive train motors use the RUN_TO_POSITION mode
     */
    public void enableEncodersToPosition() {
        fLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    /***
     * sets the drive train mobor mode to RUN_WITHOUT_ENCODER
     */
    public void disableMotorEncoders() {
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
        fLeftMotor.setTargetPosition(forward + right + spin);
        fRightMotor.setTargetPosition(forward - right - spin);
        bLeftMotor.setTargetPosition(forward - right + spin);
        bRightMotor.setTargetPosition(forward + right - spin);
    }

    /***
     * sets power of drive train motors to a value, for run to position mode
     *
     * @param power power for all motors
     */
    public void setMotorPower(double power) {
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
        return fLeftMotor.isBusy() || fRightMotor.isBusy() || bLeftMotor.isBusy() || bRightMotor.isBusy();
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

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
        buttonPusher.setPosition(0.5);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        this.resetMotorEncoders();
        this.enableMotorEncoders();
        this.disableMotorBreak();

    }

    /***
     * pushButton is a function to move the button pusher servo to press either one of the buttons, or reset its position.
     *
     * @param button input 1 for the left button, 2 for the right button, and any other number to reset it back to center
     */
    public void pushButton(int button) {
        switch (button) {
            case 1:
                this.buttonPusher.setPosition(0.3);//right button
                break;
            case -1:
                this.buttonPusher.setPosition(0.7);//left button
                break;
            default:
                buttonPusher.setPosition(0.5);
                break;
        }
    }

    /***
     * code that will one day detect the color of the beacon
     *
     * @return 1 for red, -1 for blue, 0 if can't tell
     */
    public int detectColor() {
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
        return Math.pow(input, 1.8);
    }

}

