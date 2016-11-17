package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.hardware.adafruit.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Locale;

import static org.firstinspires.ftc.teamcode.measurements.mmPerInch;
import static org.firstinspires.ftc.teamcode.measurements.pi;

/***
 * robotconfig is a simple and effective way to import the robot configuration information into every program.
 * To enable it, simply add the code
 *
 * @code robotconfig robot = new robotconfig();
 * into the op mode file.
 * Every new motor and sensor should be added to this file.
 */
public class robotconfig {

    static public DataLogger dl;
    DcMotor fLeftMotor;
    DcMotor fRightMotor;
    DcMotor bLeftMotor;
    DcMotor bRightMotor;
    Telemetry ltelemetry;
    TouchSensor touchBeacon;
    MultiplexColorSensor muxColor;
    int[] ports = {2, 5};
    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;
    DcMotor spinner;
    private Servo buttonPusher;
    private DeviceInterfaceModule cdim;
    private HardwareMap hwMap = null;
    public static LinearOpMode theLinearOpMode;
    private OpMode opMode;
    public static boolean debugMode = false;
    //color sensor code with multiplexor
    private int colorSensorLineThreashold = 3000;
    // The IMU sensor object
    private BNO055IMU imu;
    private boolean lineIsFirstTimeDebug = true;


    /* Constructor */
    public robotconfig() {

    }

    static void addlog(DataLogger dli, String function, String message) {
        dli.addField(function);
        dli.addField(message);
        dli.newLine();
    }

    /***
     * enables the motor break for the drive train motors.
     * This function only has to be run if one wants the break to be enabled.
     */
    void enableMotorBreak() {
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
     * sets max motor speed to a specific integer representing 4480 pulses per second
     */
    private void setMaxMotorSpeed() {
        addlog(dl, "robot", "setMaxMotorSpeed was called");
        if (debugMode) {
            return;
        }
        fLeftMotor.setMaxSpeed(4480);
        fRightMotor.setMaxSpeed(4480);
        bLeftMotor.setMaxSpeed(4480);
        bRightMotor.setMaxSpeed(4480);
    }

    /***
     * disables the motor break for the drive train motors.
     * This function only has to be run if one wants disable the break after it is manually enabled
     */
    void disableMotorBreak() {
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
    void resetMotorEncoders() {
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
    void enableMotorEncoders() {
        addlog(dl, "robot", "enableMotorEncoders was called");
        if (debugMode) {
            return;
        }
        fLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /***
     * makes the drive train motors use the RUN_TO_POSITION mode
     */
    void enableEncodersToPosition() {
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
    void setMotorTargets(int forward, int right, int spin) {
        addlog(dl, "robot", "setMotorTargets was called - f:r:s: " + String.format(Locale.ENGLISH, "%d", forward) + " : " + String.format(Locale.ENGLISH, "%d", right) + " : " + String.format(Locale.ENGLISH, "%d", spin));
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
    void setMotorPower(double power) {
        addlog(dl, "robot", "setMotorPower was called - power: " + String.format(Locale.ENGLISH, "%.2f", power));
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
    boolean isMotorBusy() {
        addlog(dl, "robot", "isMotorBusy was called, returning " + String.valueOf(!debugMode && (fLeftMotor.isBusy() || fRightMotor.isBusy() || bLeftMotor.isBusy() || bRightMotor.isBusy())));
        return (!debugMode && (fLeftMotor.isBusy() || fRightMotor.isBusy() || bLeftMotor.isBusy() || bRightMotor.isBusy()));
    }

    /***
     * gets the average of the 4 drivetrain motor encoders
     *
     * @return the average of the encoders as an int
     */
    int getMotorEncoderAverage() {

        if (debugMode) {
            addlog(dl, "robot", "motorEncoderAverage was called, positions unknown ");
            return (0);
        } else {
            addlog(dl, "robot", "motorEncoderAverage was called, positions: " + String.format(Locale.ENGLISH, "%d, %d, %d, %d", fLeftMotor.getCurrentPosition(), fRightMotor.getCurrentPosition(), bLeftMotor.getCurrentPosition(), bRightMotor.getCurrentPosition()));
            return (fLeftMotor.getCurrentPosition() + fRightMotor.getCurrentPosition() + bLeftMotor.getCurrentPosition() + bRightMotor.getCurrentPosition()) / 4;
        }
    }

    /* Initialize standard Hardware interfaces - LinearOpMode */

    public void init(LinearOpMode linearOpMode) {

        theLinearOpMode = linearOpMode;

        // Save reference to Hardware map in class variable

        hwMap = linearOpMode.hardwareMap;

        dl = new DataLogger("10635", "autonomousTest", theLinearOpMode.telemetry);
        addlog(dl, "r.init", "r.init was invoked (a)");
        addlog(dl, "r.init", "debug mode is " + debugMode);

        // Define and Initialize Motors
        if (!debugMode) {

            fLeftMotor = hwMap.dcMotor.get("fl_drive");
            fRightMotor = hwMap.dcMotor.get("fr_drive");
            bLeftMotor = hwMap.dcMotor.get("bl_drive");
            bRightMotor = hwMap.dcMotor.get("br_drive");

            //spinner = hwMap.dcMotor.get("spinner");

            //get sensor stuff
            cdim = hwMap.deviceInterfaceModule.get("dim");
            int milliSeconds = 48;
            muxColor = new MultiplexColorSensor(hwMap, "mux", "ada", ports, milliSeconds, MultiplexColorSensor.GAIN_16X);
            muxColor.startPolling();

            //initialize sensor stuff
//            cdim.setDigitalChannelMode(1, DigitalChannelController.Mode.OUTPUT);
//            cdim.setDigitalChannelState(1, bLedOn);

            // Set up the parameters with which we will use our IMU. Note that integration
            // algorithm here just reports accelerations to the logcat log; it doesn't actually
            // provide positional information.
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
            parameters.loggingEnabled = false;
            parameters.loggingTag = "IMU";
            parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

            // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
            // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
            // and named "imu".
            imu = hwMap.get(BNO055IMU.class, "imu");
            imu.initialize(parameters);

            touchBeacon = hwMap.touchSensor.get("touchBeacon");

            // And initialize servo
            buttonPusher = hwMap.servo.get("buttonPusher");

            fLeftMotor.setDirection(DcMotor.Direction.FORWARD); //forward for andymark motor in drivetrain
            bLeftMotor.setDirection(DcMotor.Direction.FORWARD); //forward for andymark motor in drivetrain
            fRightMotor.setDirection(DcMotor.Direction.REVERSE); //reverse for andymark motor in drivetrain
            bRightMotor.setDirection(DcMotor.Direction.REVERSE); //reverse for andymark motor in drivetrain

            //spinner.setDirection(DcMotor.Direction.REVERSE);

            // Set all motors to zero power
            fLeftMotor.setPower(0);
            fRightMotor.setPower(0);
            bLeftMotor.setPower(0);
            bRightMotor.setPower(0);

            //setMaxMotorSpeed();

            spinner.setPower(0);
        }
        //and check servo power
        this.pushButton(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        this.resetMotorEncoders();
        Thread.yield();
        this.enableMotorEncoders();

        addlog(dl, "r.init", "r.init finished (a)");
    }

        /* Initialize standard Hardware interfaces */

    public void init(OpMode opMode) {
        this.opMode = opMode;

        // Save reference to Hardware map in class variable

        hwMap = opMode.hardwareMap;

        dl = new DataLogger("10635", "autonomousTest", opMode.telemetry);
        addlog(dl, "r.init", "r.init was invoked (a)");

        // Define and Initialize Motors
        if (!debugMode) {

            fLeftMotor = hwMap.dcMotor.get("fl_drive");
            fRightMotor = hwMap.dcMotor.get("fr_drive");
            bLeftMotor = hwMap.dcMotor.get("bl_drive");
            bRightMotor = hwMap.dcMotor.get("br_drive");

            spinner = hwMap.dcMotor.get("spinner");

            //get sensor stuff
            cdim = hwMap.deviceInterfaceModule.get("dim");
            int milliSeconds = 48;
            muxColor = new MultiplexColorSensor(hwMap, "mux", "ada", ports, milliSeconds, MultiplexColorSensor.GAIN_16X);
            muxColor.startPolling();

            //initialize sensor stuff
//            cdim.setDigitalChannelMode(1, DigitalChannelController.Mode.OUTPUT);
//            cdim.setDigitalChannelState(1, bLedOn);

            // Set up the parameters with which we will use our IMU. Note that integration
            // algorithm here just reports accelerations to
            // the logcat log; it doesn't actually
            // provide positional information.
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
            parameters.loggingEnabled = false;
            parameters.loggingTag = "IMU";
            parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

            // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
            // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
            // and named "imu".
            imu = hwMap.get(BNO055IMU.class, "imu");
            imu.initialize(parameters);

            touchBeacon = hwMap.touchSensor.get("touchBeacon");

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

            //spinner.setPower(0);
        }
        //and check servo power
        this.pushButton(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        this.resetMotorEncoders();
        Thread.yield();
        this.enableMotorEncoders();

        addlog(dl, "r.init", "r.init finished (a)");
    }

    /***
     * is a shortcut to get the intrinsic z angle of the robot
     *
     * @return imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX).firstAngle;
     */
    double getCurrentAngle() {
        addlog(dl, "robot", "getCurrentAngle was invoked, returning 45");
        if (debugMode) {
            return (45);
        }

        return imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX).firstAngle;
    }

    /***
     * pushButton is a function to move the button pusher servo to press either one of the buttons, or reset its position.
     *
     * @param button input 1 for the left button, 2 for the right button, and any other number to reset it back to center
     */
    void pushButton(int button) {
        addlog(dl, "robot", "pushButton was invoked");
        if (debugMode) {
            return;
        }

        switch (button) {
            case 1:
                buttonPusher.setPosition(1);//0.72);//right button
                break;
            case -1:
                buttonPusher.setPosition(0);//0.42);//left
                break;
            default:
                buttonPusher.setPosition(0.54);
                break;
        }//*/
    }

    /***
     * code that will one day detect the color of the beacon
     *
     * @return 1 for red, -1 for blue, 0 if can't tell
     */
    int detectColor() {

        addlog(dl, "robot", "detectColor was called");
        if (debugMode) {
            return (0);
        }
        if (muxColor.getCRGB(ports[0])[1] > muxColor.getCRGB(ports[0])[3]) {
            return 1;
        } else if (muxColor.getCRGB(ports[0])[3] > muxColor.getCRGB(ports[0])[1]) {
            return -1;
        } else {
            return 0;
        }
    }

    /***
     * function is used to find the line on the field
     *
     * @return true if line is detected
     */
    boolean detectLine() {

        addlog(dl, "robot", "detectLine was called");

        if (robotconfig.debugMode) {
            if (lineIsFirstTimeDebug) {
                robotconfig.addlog(dl, "in detectLine", "returning true");
                return (true);
            } else {
                lineIsFirstTimeDebug = true;
                robotconfig.addlog(dl, "in detectLine", "returning false");
                return (false);
            }
        } else {
            robotconfig.addlog(dl, "in detectLine", "returning " + (muxColor.getCRGB(ports[1])[2] > colorSensorLineThreashold));
            return (muxColor.getCRGB(ports[1])[2] > colorSensorLineThreashold);
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

        addlog(dl, "robot", "move was called - f:r:s: " + String.format(Locale.ENGLISH, "%.2f", forward) + " : " + String.format(Locale.ENGLISH, "%.2f", right) + " : " + String.format(Locale.ENGLISH, "%.2f", spin));

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
    private double scale(double input) {

        double sign = 1.0;
        double output;

        if (input < 0.0) {
            sign = -1.0;    // remember incoming joystick direction, -1 is fully up
        }

        input = input * input;  // power transfer curve, adjust sign handling if sign is preserved by this function

        double DEAD_ZONE = 0.02;
        if (input < (DEAD_ZONE * DEAD_ZONE)) { // need to square DEAD_ZONE since input is already squared
            return (0);
        }

        output = (input - (DEAD_ZONE * DEAD_ZONE));             // shift so that range is from 0 to 1.0-DEAD_ZONE^2 instead of DEAD_ZONE^2 to 1.0
        output = output / (1.0 - (DEAD_ZONE * DEAD_ZONE));     // scale so 0 to 1.0-DEAD_ZONE^2 is now 0 to 1
        double MAX_POWER = 1.00;
        double MIN_POWER = 0.05;
        output = output * (MAX_POWER - MIN_POWER);              // scale so range is now 0 to (MAX-MIN)
        output = output + MIN_POWER;                            // shift so range is now MIN_POWER to MAX_POWER
        return (output * sign);         // don't forget to restore the sign of the input
    }

}

