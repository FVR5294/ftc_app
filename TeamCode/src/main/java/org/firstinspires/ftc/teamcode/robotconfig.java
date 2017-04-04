package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.hardware.adafruit.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Locale;

/***
 * robotconfig is a simple and effective way to import the robot configuration information into every program.
 * To enable it, simply add the code
 *
 * @code robotconfig robot = new robotconfig();
 * into the op mode file.c
 * Every new motor, servo, and sensor should be added to this file.
 */
public class robotconfig {

    static final double hyp = Math.sqrt(Math.pow(26.4, 2) + Math.pow(29, 2)) / 2;
    static public DataLogger dl;
    static LinearOpMode theLinearOpMode;
    static boolean debugMode = false;
    static double mmPerInch = measurements.mmPerInch;
    public OpticalDistanceSensor ods;
    public ColorSensor ada;
    public ModernRoboticsI2cRangeSensor ultra;
    public boolean ultraEnabled = true;
    public boolean autoIntake = true;
    public boolean intakeSensorRedundancy = true;
    public boolean eject = true;


    public ElapsedTime puncherTimer = new ElapsedTime();
    public boolean puncherTime = false;
    public int puncherDeadzone = 600;
    public ColorSensor intake;
    double maxGyro = 1;
    double minGyro = -maxGyro;
    double maxUltra = 1;
    double minUltra = -maxUltra;
    double ultraGain = 0.08;
    double gyroGain = 0.04;
    DcMotor fLeftMotor;
    DcMotor fRightMotor;
    DcMotor bLeftMotor;
    DcMotor bRightMotor;
    Telemetry ltelemetry;
    TouchSensor touchBeacon;
    TouchSensor intake1;
    TouchSensor intake2;
    TouchSensor intake3;
    TouchSensor intake1b;
    TouchSensor intake2b;
    TouchSensor intake3b;
    //    TouchSensor intake4;
    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;
    DcMotor spinner;
    DcMotor reeler;
    Servo tilt;
    Servo capLeft;
    Servo capRight;
    Servo buttonPusher;
    Servo theHammerOfDawn;
    ServoController capServoController;
    TouchSensor garry;
    Servo lvex;
    Servo rvex;
    DcMotor puncher;
    // The IMU sensor object
    BNO055IMU gyro;
    private int fLeftMotorTarget = 0;
    private int fRightMotorTarget = 0;
    private int bLeftMotorTarget = 0;
    private int bRightMotorTarget = 0;
    private int bettermovedeadzone = 100;
    private DeviceInterfaceModule cdim;
    private HardwareMap hwMap = null;
    private OpMode opMode;
    //color sensor code with multiplexor
    private double colorSensorLineThreashold = 0.2;
    private boolean lineIsFirstTimeDebug = true;

    /* Constructor */
    public robotconfig() {

    }

    static void addlog(DataLogger dli, String function, String message) {
        dli.addField(function);
        dli.addField(message);
        dli.newLine();
    }

    static void addlog(DataLogger dli, String message) {
        dli.addField(message);
        dli.newLine();
    }

    public void resetPuncherTime() {
        puncherTimer.reset();
        puncherTime = true;
    }

    public void checkPuncherTime() {
        if (puncherTimer.seconds() > 3) {
            puncherTime = false;
        }
    }

    public boolean intake(int i) {
        if (intakeSensorRedundancy)
            switch (i) {
                case 1:
                    return !intake1.isPressed() || !intake1b.isPressed();
                case 2:
                    return !intake2.isPressed() || !intake2b.isPressed();
                case 3:
                    return !intake3.isPressed() || !intake3b.isPressed();
            }
        else
            switch (i) {
                case 1:
                    return !intake1.isPressed();
                case 2:
                    return !intake2.isPressed();
                case 3:
                    return !intake3.isPressed();
            }
        return false;
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
     * sets the target position of all the motors in pulses. probably won't work as expected if less than 2 values are equal to zero
     * adds value of current position
     *
     * @param forward pulses in the forward direction
     * @param right   pulses in sliding to the right (or left if negative)
     * @param spin    pulses in spinning clockwise
     */
    void setMyMotorTargets(int forward, int right, int spin) {
        addlog(dl, "robot", "setMotorTargets was called - f:r:s: " + String.format(Locale.ENGLISH, "%d", forward) + " : " + String.format(Locale.ENGLISH, "%d", right) + " : " + String.format(Locale.ENGLISH, "%d", spin));
        if (debugMode) {
            return;
        }
        fLeftMotorTarget = (fLeftMotor.getCurrentPosition() + forward + right + spin);
        fRightMotorTarget = (fRightMotor.getCurrentPosition() + forward - right - spin);
        bLeftMotorTarget = (bLeftMotor.getCurrentPosition() + forward - right + spin);
        bRightMotorTarget = (bRightMotor.getCurrentPosition() + forward + right - spin);
    }

    /**
     * Set motor targets to move robot a specific amount from the current position
     *
     * @param left  number of encoder pulses for motors on the left side
     * @param right number of encoder pulses for motors on the right side
     */
    void setMyMotorTankTargets(int left, int right) {
        addlog(dl, "robot", "setMotorTankTargets was called - L:r: " + String.format(Locale.ENGLISH, "%d", left) + " : " + String.format(Locale.ENGLISH, "%d", right));
        if (debugMode) {
            return;
        }
        fLeftMotorTarget = fLeftMotor.getCurrentPosition() + left;
        fRightMotorTarget = fRightMotor.getCurrentPosition() + right;
        bLeftMotorTarget = bLeftMotor.getCurrentPosition() + left;
        bRightMotorTarget = bRightMotor.getCurrentPosition() + right;
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


        try {
            fLeftMotor = hwMap.dcMotor.get("fl_drive");
        } catch (Exception err) {
            debugMode = false;
        }

        try {
            ultra = hwMap.get(ModernRoboticsI2cRangeSensor.class, "ultra");
        } catch (Exception err) {
            ultraEnabled = false;
            theLinearOpMode.telemetry.addData("wiring", "connect Range Sensor ultra");
        }

        try {
            puncher = hwMap.dcMotor.get("puncher");
            puncher.setDirection(DcMotor.Direction.REVERSE);
            puncher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        } catch (Exception err) {
            theLinearOpMode.telemetry.addData("wiring", "connect motor puncher");
        }

        dl = new DataLogger("10635", "autonomousTest.xml", theLinearOpMode.telemetry);
        addlog(dl, "r.init", "r.init was invoked (a)");
        addlog(dl, "r.init", "debug mode is " + debugMode);

        // Define and Initialize Motors
        if (!debugMode) {

            capServoController = hwMap.servoController.get("Servo Controller 2");
            fLeftMotor = hwMap.dcMotor.get("fl_drive");
            fRightMotor = hwMap.dcMotor.get("fr_drive");
            bLeftMotor = hwMap.dcMotor.get("bl_drive");
            bRightMotor = hwMap.dcMotor.get("br_drive");
            reeler = hwMap.dcMotor.get("Reeler");
//            reeler.setDirection(DcMotorSimple.Direction.REVERSE);
            spinner = hwMap.dcMotor.get("spinner");
            theHammerOfDawn = hwMap.servo.get("theHammerOfDawn");
            theHammerOfDawn.setPosition(0.5);
            tilt = hwMap.servo.get("Tilt");
            tilt.setDirection(Servo.Direction.REVERSE);
            tilt.setPosition(0.5);
            capLeft = hwMap.servo.get("capLeft");
            capLeft.setDirection(Servo.Direction.REVERSE);
            capLeft.setPosition(0.05);
            capRight = hwMap.servo.get("capRight");
            capRight.setPosition(0.05);

            garry = hwMap.touchSensor.get("punchLimit");

            lvex = hwMap.servo.get("lvex");
            rvex = hwMap.servo.get("rvex");

            lvex.setDirection(Servo.Direction.REVERSE);
            rvex.setDirection(Servo.Direction.REVERSE);

            lvex.setPosition(0.5);
            rvex.setPosition(0.5);

            //get sensor stuff
//            cdim = hwMap.deviceInterfaceModule.get("dim");
//            int milliSeconds = 1;       // should set to minimum, which is 2.4 ms - needs testing
//            muxColor = new MultiplexColorSensor(hwMap, "mux", "ada", ports, milliSeconds, MultiplexColorSensor.GAIN_16X);
//            muxColor.startPolling();
            ods = hwMap.opticalDistanceSensor.get("ods");
            colorSensorLineThreashold = ods.getLightDetected() * 2;
            addlog(dl, "r.init", "colorSensorLineThreshhold is " + colorSensorLineThreashold);
            ada = hwMap.colorSensor.get("ada");

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
            gyro = hwMap.get(BNO055IMU.class, "imu");
            gyro.initialize(parameters);

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

            spinner.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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

        try {
            fLeftMotor = hwMap.dcMotor.get("fl_drive");
        } catch (Exception err) {
            debugMode = false;
        }

        try {
            ultra = hwMap.get(ModernRoboticsI2cRangeSensor.class, "ultra");
        } catch (Exception err) {
            ultraEnabled = false;
            opMode.telemetry.addData("wiring", "connect Range Sensor ultra");
        }

        try {
            puncher = hwMap.dcMotor.get("puncher");
            puncher.setDirection(DcMotor.Direction.REVERSE);
            puncher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        } catch (Exception err) {
            opMode.telemetry.addData("wiring", "connect motor puncher");
        }

        try {
            intake1 = hwMap.touchSensor.get("intake1");
            intake2 = hwMap.touchSensor.get("intake2");
            intake3 = hwMap.touchSensor.get("intake3");
        } catch (Exception err) {
            autoIntake = false;
            opMode.telemetry.addData("wiring", "connect touch sensors intake[1-4]");
        }

        try {
            intake1b = hwMap.touchSensor.get("intake1b");
            intake2b = hwMap.touchSensor.get("intake2b");
            intake3b = hwMap.touchSensor.get("intake3b");
        } catch (Exception err) {
            intakeSensorRedundancy = false;
            opMode.telemetry.addData("wiring", "connect redundant touch sensors intake[1-4]b");
        }

        try {
            intake = hwMap.colorSensor.get("intake");
        } catch (Exception err) {
            eject = false;
            opMode.telemetry.addData("wiring", "connect color sensor intake");
        }

        dl = new DataLogger("10635", "teleopTest.csv", opMode.telemetry);
        addlog(dl, "r.init", "r.init was invoked (a)");
        addlog(dl, "r.init", "debug mode is " + debugMode);

        // Define and Initialize Motors
        if (!debugMode) {

            fLeftMotor = hwMap.dcMotor.get("fl_drive");
            fRightMotor = hwMap.dcMotor.get("fr_drive");
            bLeftMotor = hwMap.dcMotor.get("bl_drive");
            bRightMotor = hwMap.dcMotor.get("br_drive");
            reeler = hwMap.dcMotor.get("Reeler");
//            reeler.setDirection(DcMotorSimple.Direction.REVERSE);
            spinner = hwMap.dcMotor.get("spinner");
            theHammerOfDawn = hwMap.servo.get("theHammerOfDawn");
            theHammerOfDawn.setPosition(0.5);
            tilt = hwMap.servo.get("Tilt");
            tilt.setDirection(Servo.Direction.REVERSE);
            capLeft = hwMap.servo.get("capLeft");
            capLeft.setDirection(Servo.Direction.REVERSE);
            capLeft.setPosition(0.05);
            capRight = hwMap.servo.get("capRight");
            tilt.setPosition(0.5);
            capRight.setPosition(0.05);

            garry = hwMap.touchSensor.get("punchLimit");

            lvex = hwMap.servo.get("lvex");
            rvex = hwMap.servo.get("rvex");

            lvex.setDirection(Servo.Direction.REVERSE);
            rvex.setDirection(Servo.Direction.REVERSE);

            lvex.setPosition(0.5);
            rvex.setPosition(0.5);

            //get sensor stuff
            cdim = hwMap.deviceInterfaceModule.get("dim");
//            int milliSeconds = 1;       // should set to minimum, which is 2.4 ms - needs testing
//            muxColor = new MultiplexColorSensor(hwMap, "mux", "ada", ports, milliSeconds, MultiplexColorSensor.GAIN_16X);
//            muxColor.startPolling();
//           ods = hwMap.opticalDistanceSensor.get("ods");
//            colorSensorLineThreashold = ods.getLightDetected() * 2;
//            addlog(dl, "r.init", "colorSensorLineThreshhold is " + colorSensorLineThreashold);
//            ada = hwMap.colorSensor.get("ada");

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
            gyro = hwMap.get(BNO055IMU.class, "imu");
            gyro.initialize(parameters);

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

            spinner.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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

    /***
     * is a shortcut to get the intrinsic z angle of the robot
     *
     * @return gyro.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX).firstAngle;
     */
    double getCurrentAngle() {
        addlog(dl, "robot", "getCurrentAngle was invoked, returning 45");
        if (debugMode) {
            return (45);
        }

        return gyro.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX).firstAngle;
    }

    /***
     * pushButton is a function to move the button pusher servo to press either one of the buttons, or reset its position.
     *
     * @param button input -1 for the left button, 1 for the right button, and any other number to reset it back to center
     */
    void pushButton(int button) {
        addlog(dl, "robot", String.format(Locale.ENGLISH, "pushButton was invoked with button %d", button));
        if (debugMode) {
            return;
        }

        switch (button) {
            case 1:
                buttonPusher.setPosition(1);//right button
                break;
            case -1:
                buttonPusher.setPosition(0);//left
                break;
            default:
                buttonPusher.setPosition(0.5);
                break;
        }//*/
    }

    /***
     * pushSoftButton is a function to move the button pusher servo to press either one of the buttons, or reset its position.
     * like the name says, it is softer than pushButton and less likely to burn out a servo if left on for long periods of time.
     *
     * @param button input -1 for the left button, 1 for the right button, and any other number to reset it back to center
     */
    void pushSoftButton(int button) {
        addlog(dl, "robot", String.format(Locale.ENGLISH, "pushButton was invoked with button %d", button));
        if (debugMode) {
            return;
        }

        switch (button) {
            case 1:
                buttonPusher.setPosition(0.7);//right button
                break;
            case -1:
                buttonPusher.setPosition(0.3);//left
                break;
            default:
                buttonPusher.setPosition(0.5);
                break;
        }//*/
    }

    /***
     * code that detects the color of the left side of the beacon
     *
     * @return 1 for red, -1 for blue, 0 if can't tell
     */
    int detectColor() {

        addlog(dl, "robot", "detectColor was called");
        if (debugMode) {
            return (0);
        }
        robotconfig.addlog(dl, "robot", String.format(Locale.ENGLISH, "ada.red, %d, ada.blue, %d", ada.red(), ada.blue()));
        if (ada.red() > ada.blue()) {
            return 1;
        } else if (ada.red() < ada.blue()) {
            return -1;
        } else if (ada.red() > ada.blue()) {
            return 1;
        } else if (ada.red() < ada.blue()) {
            return -1;
        } else {
            return 0;
        }
    }

    /***
     * function is used to position robot over the line on the field
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
            // int colorvalue = muxColor.getCRGB(ports[1])[2];
            double colorvalue = ods.getLightDetected();
            robotconfig.addlog(dl, "in detectLine", String.format(Locale.ENGLISH, "ods.getLightDetected(), %.3f", colorvalue));
            robotconfig.addlog(dl, "in detectLine", "returning, " + (colorvalue > colorSensorLineThreashold));
            return (colorvalue > colorSensorLineThreashold);
        }

    }

//    /***
//     * possibly more responsive function is used to find the line on the field
//     *
//     * @return true if line is detected
//     */
//    boolean detectLineResponsive() {
//
//        int currentColorVal;
//        long endTime;
//
//        endTime = System.nanoTime() + 3 * 1000000;
//
//        addlog(dl, "robot", "detectLineResponsive was called");
//
//        if (robotconfig.debugMode) {
//            if (lineIsFirstTimeDebug) {
//                robotconfig.addlog(dl, "in detectLineResponsive", "returning true");
//                return (true);
//            } else {
//                lineIsFirstTimeDebug = true;
//                robotconfig.addlog(dl, "in detectLineResponsive", "returning false");
//                return (false);
//            }
//        } else {
//
//            while (System.nanoTime() < endTime) {
//                currentColorVal = muxColor.getCRGB(ports[1])[2];
//                robotconfig.addlog(colorsensors, "in detectLineResponsive", "found value, " + currentColorVal);
//                robotconfig.addlog(dl, "in detectLineResponsive", "found value, " + currentColorVal);
//                if (currentColorVal > colorSensorLineThreashold) {
//                    // robot.move(0, 0, 0);     // maybe turn motors off here for quickest response?
//                    return (true);
//                }
//            }
//            // timed out here, so give up - maybe we'll get the next one
//            // robot.move(0, 0, 0);     // and maybe turn motors off here as well?
//            return (true);
//        }
//
//    }

    /***
     * move is a function to efficiently set the power values of all 4 drive train motors in one quick line.
     *
     * @param forward double: ranges from 1=forward to -1=backward
     * @param right   double: ranges from 1=slide right to -1=slide left
     * @param spin    double: ranges from 1=turn clockwise to -1=turn counterclockwise
     */
    public void move(double forward, double right, double spin) {

        addlog(dl, "robot", "move was called - f:r:s:, " + String.format(Locale.ENGLISH, "%.2f", forward) + ", " + String.format(Locale.ENGLISH, "%.2f", right) + ", " + String.format(Locale.ENGLISH, "%.2f", spin));


        if (debugMode) {
            return;
        }

        fLeftMotor.setPower(forward + right + spin);
        fRightMotor.setPower(forward - right - spin);
        bLeftMotor.setPower(forward - right + spin);
        bRightMotor.setPower(forward + right - spin);

        addlog(dl, "robot", "move powers are fl:fr:bl:br, " + String.format(Locale.ENGLISH, "%.2f", fLeftMotor.getPower()) + ", " + String.format(Locale.ENGLISH, "%.2f", fRightMotor.getPower()) + ", " + String.format(Locale.ENGLISH, "%.2f", bLeftMotor.getPower()) + ", " + String.format(Locale.ENGLISH, "%.2f", bRightMotor.getPower()));

    }

    /***
     * ultramove is a function to efficiently set the power values of all 4 drive train motors in one quick line while using gyro and ultrasonic sensors
     *
     * @param forward double: ranges from 1=forward to -1=backward
     * @param right   double: ranges from 1=slide right to -1=slide left
     * @param target  double: a target distance form the wall in cm
     */
    public void ultramove(double forward, double right, double target) {

        if (debugMode) {
            return;
        }

        double ultraValue = 0;
        double gyroValue = getCurrentAngle() - (Math.round(getCurrentAngle() / 90) * 90);
        double spin = Math.max(minGyro, Math.min(maxGyro, gyroValue * gyroGain));

        if (ultraEnabled)
            ultraValue = Math.max(minUltra, Math.min(maxUltra, (ultra.cmUltrasonic() * Math.cos(Math.toRadians(gyroValue)) + hyp * Math.cos(Math.toRadians(gyroValue + 45)) - target) * ultraGain));

        double fLeftMotorPower = forward + right;
        double fRightMotorPower = forward - right;
        double bLeftMotorPower = forward - right;
        double bRightMotorPower = forward + right;

        double max = Math.max(Math.max(Math.abs(fLeftMotorPower), Math.abs(bLeftMotorPower)), Math.max(Math.abs(fRightMotorPower), Math.abs(bRightMotorPower)));

        fLeftMotorPower += (ultraValue + spin) * max;
        fRightMotorPower += (ultraValue - spin) * max;
        bLeftMotorPower += (ultraValue + spin) * max;
        bRightMotorPower += (ultraValue - spin) * max;

        fLeftMotor.setPower(fLeftMotorPower);
        fRightMotor.setPower(fRightMotorPower);
        bLeftMotor.setPower(bLeftMotorPower);
        bRightMotor.setPower(bRightMotorPower);

        addlog(dl, "robot", "ultramove powers are fl:fr:bl:br, " + String.format(Locale.ENGLISH, "%.2f", fLeftMotor.getPower()) + ", " + String.format(Locale.ENGLISH, "%.2f", fRightMotor.getPower()) + ", " + String.format(Locale.ENGLISH, "%.2f", bLeftMotor.getPower()) + ", " + String.format(Locale.ENGLISH, "%.2f", bRightMotor.getPower()));

    }

    /***
     * ultramove is a function to efficiently set the power values of all 4 drive train motors in one quick line while using gyro sensor
     *
     * @param forward double: ranges from 1=forward to -1=backward
     * @param right   double: ranges from 1=slide right to -1=slide left
     */
    public void ultramove(double forward, double right) {

        if (debugMode) {
            return;
        }

        double ultraValue = 0;
        double gyroValue = getCurrentAngle() - (Math.round(getCurrentAngle() / 90) * 90);
        double spin = Math.max(minGyro, Math.min(maxGyro, gyroValue * gyroGain));

        double fLeftMotorPower = forward + right;
        double fRightMotorPower = forward - right;
        double bLeftMotorPower = forward - right;
        double bRightMotorPower = forward + right;

        double max = Math.max(Math.max(Math.abs(fLeftMotorPower), Math.abs(bLeftMotorPower)), Math.max(Math.abs(fRightMotorPower), Math.abs(bRightMotorPower)));

        fLeftMotorPower += (ultraValue + spin) * max;
        fRightMotorPower += (ultraValue - spin) * max;
        bLeftMotorPower += (ultraValue + spin) * max;
        bRightMotorPower += (ultraValue - spin) * max;

        fLeftMotor.setPower(fLeftMotorPower);
        fRightMotor.setPower(fRightMotorPower);
        bLeftMotor.setPower(bLeftMotorPower);
        bRightMotor.setPower(bRightMotorPower);

        addlog(dl, "robot", "ultramove powers are fl:fr:bl:br, " + String.format(Locale.ENGLISH, "%.2f", fLeftMotor.getPower()) + ", " + String.format(Locale.ENGLISH, "%.2f", fRightMotor.getPower()) + ", " + String.format(Locale.ENGLISH, "%.2f", bLeftMotor.getPower()) + ", " + String.format(Locale.ENGLISH, "%.2f", bRightMotor.getPower()));

    }

    /***
     * bettermove is a function to efficiently set the power values of all 4 drive train motors in a way to get all 4 motors to their targets at the same time as fast as possible
     * similar to ultramove but without gyro and ultrasonic sensors
     */
    public void bettermove() {


        if (debugMode) {
            return;
        }

        double fLeftMotorPower = (fLeftMotorTarget - fLeftMotor.getCurrentPosition());
        double fRightMotorPower = (fRightMotorTarget - fRightMotor.getCurrentPosition());
        double bLeftMotorPower = (bLeftMotorTarget - bLeftMotor.getCurrentPosition());
        double bRightMotorPower = (bRightMotorTarget - bRightMotor.getCurrentPosition());
        double max = Math.max(Math.max(Math.abs(fLeftMotorPower), Math.abs(bLeftMotorPower)), Math.max(Math.abs(fRightMotorPower), Math.abs(bRightMotorPower)));

        if (Math.abs(fLeftMotorPower) < bettermovedeadzone)
            fLeftMotorPower = 0;
        if (Math.abs(fRightMotorPower) < bettermovedeadzone)
            fRightMotorPower = 0;
        if (Math.abs(bRightMotorPower) < bettermovedeadzone)
            bRightMotorPower = 0;
        if (Math.abs(bLeftMotorPower) < bettermovedeadzone)
            bLeftMotorPower = 0;

        max /= Math.min(1, Math.max(Math.abs(max / 3000), 0.5));

        if (max > 0) {
            fLeftMotor.setPower(fLeftMotorPower / max);
            fRightMotor.setPower(fRightMotorPower / max);
            bLeftMotor.setPower(bLeftMotorPower / max);
            bRightMotor.setPower(bRightMotorPower / max);
        } else {
            fLeftMotor.setPower(0);
            fRightMotor.setPower(0);
            bLeftMotor.setPower(0);
            bRightMotor.setPower(0);
        }

        addlog(dl, "robot", "bettermove powers are fl:fr:bl:br, " + String.format(Locale.ENGLISH, "%.2f", fLeftMotor.getPower()) + ", " + String.format(Locale.ENGLISH, "%.2f", fRightMotor.getPower()) + ", " + String.format(Locale.ENGLISH, "%.2f", bLeftMotor.getPower()) + ", " + String.format(Locale.ENGLISH, "%.2f", bRightMotor.getPower()));

    }

    /***
     * ultra is a function to efficiently set the power values of all 4 drive train motors in a way to get all 4 motors to their targets at the same time as fast as possible
     * similar to bettermove but with gyro and ultrasonic sensors
     * gyro corrects rotation and ultrasonic prevents running into or drifting away from the wall
     * @param target a measurement of the target distance between the center of the robot and the wall in cm
     */
    public void ultramove(double target) {


        if (debugMode) {
            return;
        }

        double ultraValue = 0;
        double gyroValue = getCurrentAngle() - (Math.round(getCurrentAngle() / 90) * 90);
        double spin = Math.max(minGyro, Math.min(maxGyro, gyroValue * gyroGain));

        if (ultraEnabled)
            ultraValue = Math.max(minUltra, Math.min(maxUltra, (ultra.cmUltrasonic() * Math.cos(Math.toRadians(gyroValue)) + hyp * Math.cos(Math.toRadians(gyroValue + 45)) - target) * ultraGain));

        double fLeftMotorPower = (fLeftMotorTarget - fLeftMotor.getCurrentPosition());
        double fRightMotorPower = (fRightMotorTarget - fRightMotor.getCurrentPosition());
        double bLeftMotorPower = (bLeftMotorTarget - bLeftMotor.getCurrentPosition());
        double bRightMotorPower = (bRightMotorTarget - bRightMotor.getCurrentPosition());
        double max = Math.max(Math.max(Math.abs(fLeftMotorPower), Math.abs(bLeftMotorPower)), Math.max(Math.abs(fRightMotorPower), Math.abs(bRightMotorPower)));
        fLeftMotorPower += (ultraValue + spin) * max;
        fRightMotorPower += (ultraValue - spin) * max;
        bLeftMotorPower += (ultraValue + spin) * max;
        bRightMotorPower += (ultraValue - spin) * max;

        if (Math.abs(fLeftMotorPower) < bettermovedeadzone)
            fLeftMotorPower = 0;
        if (Math.abs(fRightMotorPower) < bettermovedeadzone)
            fRightMotorPower = 0;
        if (Math.abs(bRightMotorPower) < bettermovedeadzone)
            bRightMotorPower = 0;
        if (Math.abs(bLeftMotorPower) < bettermovedeadzone)
            bLeftMotorPower = 0;

        max = Math.max(Math.max(Math.abs(fLeftMotorPower), Math.abs(bLeftMotorPower)), Math.max(Math.abs(fRightMotorPower), Math.abs(bRightMotorPower)));

        max /= Math.min(1, Math.max(Math.abs(max / 3000), 0.5));

        if (max > 0) {
            fLeftMotor.setPower(fLeftMotorPower / max);
            fRightMotor.setPower(fRightMotorPower / max);
            bLeftMotor.setPower(bLeftMotorPower / max);
            bRightMotor.setPower(bRightMotorPower / max);
        } else {
            fLeftMotor.setPower(0);
            fRightMotor.setPower(0);
            bLeftMotor.setPower(0);
            bRightMotor.setPower(0);
        }

        addlog(dl, "robot", "bettermove powers are fl:fr:bl:br, " + String.format(Locale.ENGLISH, "%.2f", fLeftMotor.getPower()) + ", " + String.format(Locale.ENGLISH, "%.2f", fRightMotor.getPower()) + ", " + String.format(Locale.ENGLISH, "%.2f", bLeftMotor.getPower()) + ", " + String.format(Locale.ENGLISH, "%.2f", bRightMotor.getPower()));

    }

    /***
     * like ultramove but before the gyro was used with the range sensor because it is impressive how drunk it moves
     *
     * @param target target distance to wall in cm
     */
    public void drunkmove(double target) {


        if (debugMode) {
            return;
        }

        double ultraValue = 0;
        double gyroValue = getCurrentAngle() - (Math.round(getCurrentAngle() / 90) * 90);
        double spin = Math.max(minGyro, Math.min(maxGyro, gyroValue * gyroGain));

        if (ultraEnabled)
            ultraValue = Math.max(minUltra, Math.min(maxUltra, (ultra.cmUltrasonic() - target) * ultraGain));

        double fLeftMotorPower = (fLeftMotorTarget - fLeftMotor.getCurrentPosition());
        double fRightMotorPower = (fRightMotorTarget - fRightMotor.getCurrentPosition());
        double bLeftMotorPower = (bLeftMotorTarget - bLeftMotor.getCurrentPosition());
        double bRightMotorPower = (bRightMotorTarget - bRightMotor.getCurrentPosition());
        double max = Math.max(Math.max(Math.abs(fLeftMotorPower), Math.abs(bLeftMotorPower)), Math.max(Math.abs(fRightMotorPower), Math.abs(bRightMotorPower)));
        fLeftMotorPower += (ultraValue + spin) * max;
        fRightMotorPower += (ultraValue - spin) * max;
        bLeftMotorPower += (ultraValue + spin) * max;
        bRightMotorPower += (ultraValue - spin) * max;

        if (Math.abs(fLeftMotorPower) < bettermovedeadzone)
            fLeftMotorPower = 0;
        if (Math.abs(fRightMotorPower) < bettermovedeadzone)
            fRightMotorPower = 0;
        if (Math.abs(bRightMotorPower) < bettermovedeadzone)
            bRightMotorPower = 0;
        if (Math.abs(bLeftMotorPower) < bettermovedeadzone)
            bLeftMotorPower = 0;

        max = Math.max(Math.max(Math.abs(fLeftMotorPower), Math.abs(bLeftMotorPower)), Math.max(Math.abs(fRightMotorPower), Math.abs(bRightMotorPower)));

        max /= Math.min(1, Math.max(Math.abs(max / 3000), 0.5));

        if (max > 0) {
            fLeftMotor.setPower(fLeftMotorPower / max);
            fRightMotor.setPower(fRightMotorPower / max);
            bLeftMotor.setPower(bLeftMotorPower / max);
            bRightMotor.setPower(bRightMotorPower / max);
        } else {
            fLeftMotor.setPower(0);
            fRightMotor.setPower(0);
            bLeftMotor.setPower(0);
            bRightMotor.setPower(0);
        }

        addlog(dl, "robot", "bettermove powers are fl:fr:bl:br, " + String.format(Locale.ENGLISH, "%.2f", fLeftMotor.getPower()) + ", " + String.format(Locale.ENGLISH, "%.2f", fRightMotor.getPower()) + ", " + String.format(Locale.ENGLISH, "%.2f", bLeftMotor.getPower()) + ", " + String.format(Locale.ENGLISH, "%.2f", bRightMotor.getPower()));

    }

    /***
     * gyromove uses predefined motor targets and the gyro to move to the target and correct rotation
     *
     * @param power the power to run the fastest motor at
     */
    public void gyromove(double power) {


        if (debugMode) {
            return;
        }

        double ultraValue = 0;
        double gyroValue = getCurrentAngle() - (Math.round(getCurrentAngle() / 90) * 90);
        double spin = Math.max(minGyro, Math.min(maxGyro, gyroValue * gyroGain));

        double fLeftMotorPower = (fLeftMotorTarget - fLeftMotor.getCurrentPosition());
        double fRightMotorPower = (fRightMotorTarget - fRightMotor.getCurrentPosition());
        double bLeftMotorPower = (bLeftMotorTarget - bLeftMotor.getCurrentPosition());
        double bRightMotorPower = (bRightMotorTarget - bRightMotor.getCurrentPosition());
        double max = Math.max(Math.max(Math.abs(fLeftMotorPower), Math.abs(bLeftMotorPower)), Math.max(Math.abs(fRightMotorPower), Math.abs(bRightMotorPower)));

        if (Math.abs(fLeftMotorPower) < bettermovedeadzone)
            fLeftMotorPower = 0;
        if (Math.abs(fRightMotorPower) < bettermovedeadzone)
            fRightMotorPower = 0;
        if (Math.abs(bRightMotorPower) < bettermovedeadzone)
            bRightMotorPower = 0;
        if (Math.abs(bLeftMotorPower) < bettermovedeadzone)
            bLeftMotorPower = 0;

        fLeftMotorPower += (ultraValue + spin) * max;
        fRightMotorPower += (ultraValue - spin) * max;
        bLeftMotorPower += (ultraValue + spin) * max;
        bRightMotorPower += (ultraValue - spin) * max;

        max = Math.max(Math.max(Math.abs(fLeftMotorPower), Math.abs(bLeftMotorPower)), Math.max(Math.abs(fRightMotorPower), Math.abs(bRightMotorPower)));

        max /= power;

        if (max > 0) {
            fLeftMotor.setPower(fLeftMotorPower / max);
            fRightMotor.setPower(fRightMotorPower / max);
            bLeftMotor.setPower(bLeftMotorPower / max);
            bRightMotor.setPower(bRightMotorPower / max);
        } else {
            fLeftMotor.setPower(0);
            fRightMotor.setPower(0);
            bLeftMotor.setPower(0);
            bRightMotor.setPower(0);
        }

        addlog(dl, "robot", "bettermove powers are fl:fr:bl:br, " + String.format(Locale.ENGLISH, "%.2f", fLeftMotor.getPower()) + ", " + String.format(Locale.ENGLISH, "%.2f", fRightMotor.getPower()) + ", " + String.format(Locale.ENGLISH, "%.2f", bLeftMotor.getPower()) + ", " + String.format(Locale.ENGLISH, "%.2f", bRightMotor.getPower()));

    }

    /***
     * function to set power of motors to get them to their respective targets
     */
    public void gyromove() {


        if (debugMode) {
            return;
        }

        double ultraValue = 0;
        double gyroValue = getCurrentAngle() - (Math.round(getCurrentAngle() / 45) * 45);
        double spin = Math.max(minGyro, Math.min(maxGyro, gyroValue * gyroGain));

        double fLeftMotorPower = (fLeftMotorTarget - fLeftMotor.getCurrentPosition());
        double fRightMotorPower = (fRightMotorTarget - fRightMotor.getCurrentPosition());
        double bLeftMotorPower = (bLeftMotorTarget - bLeftMotor.getCurrentPosition());
        double bRightMotorPower = (bRightMotorTarget - bRightMotor.getCurrentPosition());
        double max = Math.max(Math.max(Math.abs(fLeftMotorPower), Math.abs(bLeftMotorPower)), Math.max(Math.abs(fRightMotorPower), Math.abs(bRightMotorPower)));

        if (Math.abs(fLeftMotorPower) < bettermovedeadzone)
            fLeftMotorPower = 0;
        if (Math.abs(fRightMotorPower) < bettermovedeadzone)
            fRightMotorPower = 0;
        if (Math.abs(bRightMotorPower) < bettermovedeadzone)
            bRightMotorPower = 0;
        if (Math.abs(bLeftMotorPower) < bettermovedeadzone)
            bLeftMotorPower = 0;

        fLeftMotorPower += (ultraValue + spin) * max;
        fRightMotorPower += (ultraValue - spin) * max;
        bLeftMotorPower += (ultraValue + spin) * max;
        bRightMotorPower += (ultraValue - spin) * max;

        max = Math.max(Math.max(Math.abs(fLeftMotorPower), Math.abs(bLeftMotorPower)), Math.max(Math.abs(fRightMotorPower), Math.abs(bRightMotorPower)));

        max /= Math.min(1, Math.max(Math.abs(max / 3000), 0.5));

        if (max > 0) {
            fLeftMotor.setPower(fLeftMotorPower / max);
            fRightMotor.setPower(fRightMotorPower / max);
            bLeftMotor.setPower(bLeftMotorPower / max);
            bRightMotor.setPower(bRightMotorPower / max);
        } else {
            fLeftMotor.setPower(0);
            fRightMotor.setPower(0);
            bLeftMotor.setPower(0);
            bRightMotor.setPower(0);
        }

        addlog(dl, "robot", "bettermove powers are fl:fr:bl:br, " + String.format(Locale.ENGLISH, "%.2f", fLeftMotor.getPower()) + ", " + String.format(Locale.ENGLISH, "%.2f", fRightMotor.getPower()) + ", " + String.format(Locale.ENGLISH, "%.2f", bLeftMotor.getPower()) + ", " + String.format(Locale.ENGLISH, "%.2f", bRightMotor.getPower()));

    }

    /***
     * move is a function to efficiently set the power values of all 4 drive train motors in one quick line.
     *
     * @param power the power to run the fastest motor at
     */
    public void bettermove(double power) {


        if (debugMode) {
            return;
        }

        double fLeftMotorPower = (fLeftMotorTarget - fLeftMotor.getCurrentPosition());
        double fRightMotorPower = (fRightMotorTarget - fRightMotor.getCurrentPosition());
        double bLeftMotorPower = (bLeftMotorTarget - bLeftMotor.getCurrentPosition());
        double bRightMotorPower = (bRightMotorTarget - bRightMotor.getCurrentPosition());
        double max = Math.max(Math.max(Math.abs(fLeftMotorPower), Math.abs(bLeftMotorPower)), Math.max(Math.abs(fRightMotorPower), Math.abs(bRightMotorPower))) / power;

        if (Math.abs(fLeftMotorPower) < bettermovedeadzone)
            fLeftMotorPower = 0;
        if (Math.abs(fRightMotorPower) < bettermovedeadzone)
            fRightMotorPower = 0;
        if (Math.abs(bRightMotorPower) < bettermovedeadzone)
            bRightMotorPower = 0;
        if (Math.abs(bLeftMotorPower) < bettermovedeadzone)
            bLeftMotorPower = 0;

        max /= power;

        if (max > 0) {
            fLeftMotor.setPower(fLeftMotorPower / max);
            fRightMotor.setPower(fRightMotorPower / max);
            bLeftMotor.setPower(bLeftMotorPower / max);
            bRightMotor.setPower(bRightMotorPower / max);
        } else {
            fLeftMotor.setPower(0);
            fRightMotor.setPower(0);
            bLeftMotor.setPower(0);
            bRightMotor.setPower(0);
        }

        addlog(dl, "robot", "bettermove powers are fl:fr:bl:br, " + String.format(Locale.ENGLISH, "%.2f", fLeftMotor.getPower()) + ", " + String.format(Locale.ENGLISH, "%.2f", fRightMotor.getPower()) + ", " + String.format(Locale.ENGLISH, "%.2f", bLeftMotor.getPower()) + ", " + String.format(Locale.ENGLISH, "%.2f", bRightMotor.getPower()));

    }

    /***
     * check if robot motors are within tolerance
     *
     * @return true if motors are close enough to the target
     */
    public boolean bettermoving() {
        int fLeftMotorPower = (fLeftMotorTarget - fLeftMotor.getCurrentPosition());
        int fRightMotorPower = (fRightMotorTarget - fRightMotor.getCurrentPosition());
        int bLeftMotorPower = (bLeftMotorTarget - bLeftMotor.getCurrentPosition());
        int bRightMotorPower = (bRightMotorTarget - bRightMotor.getCurrentPosition());
        return Math.abs(fLeftMotorPower) < bettermovedeadzone && Math.abs(fRightMotorPower) < bettermovedeadzone && Math.abs(bLeftMotorPower) < bettermovedeadzone && Math.abs(bRightMotorPower) < bettermovedeadzone;
    }

    /***
     * check if robot motors are within tolerance factoring in range and gyro sensors
     *
     * @param target distance from wall in cm
     * @return true if motors are close enough to the target
     */
    public boolean ultramoving(double target) {

        double ultraValue = 0;
        double gyroValue = getCurrentAngle() - (Math.round(getCurrentAngle() / 90) * 90);
        double spin = Math.max(minGyro, Math.min(maxGyro, gyroValue * gyroGain));

        if (ultraEnabled)
            ultraValue = Math.max(minUltra, Math.min(maxUltra, (ultra.cmUltrasonic() * Math.cos(Math.toRadians(gyroValue)) + hyp * Math.cos(Math.toRadians(gyroValue + 45)) - target) * ultraGain));
        int fLeftMotorPower = (fLeftMotorTarget - fLeftMotor.getCurrentPosition());
        int fRightMotorPower = (fRightMotorTarget - fRightMotor.getCurrentPosition());
        int bLeftMotorPower = (bLeftMotorTarget - bLeftMotor.getCurrentPosition());
        int bRightMotorPower = (bRightMotorTarget - bRightMotor.getCurrentPosition());
        double max = Math.max(Math.max(Math.abs(fLeftMotorPower), Math.abs(bLeftMotorPower)), Math.max(Math.abs(fRightMotorPower), Math.abs(bRightMotorPower)));
        fLeftMotorPower += (ultraValue + spin) * max;
        fRightMotorPower += (ultraValue - spin) * max;
        bLeftMotorPower += (ultraValue + spin) * max;
        bRightMotorPower += (ultraValue - spin) * max;
        return Math.abs(fLeftMotorPower) < bettermovedeadzone && Math.abs(fRightMotorPower) < bettermovedeadzone && Math.abs(bLeftMotorPower) < bettermovedeadzone && Math.abs(bRightMotorPower) < bettermovedeadzone;
    }

    /***
     * function checks if motors are close enough to their targets factoring the range sensor and gyro sensor
     *
     * @param target distance to wall in cm
     * @return true if motors are close enough to the target
     */
    public boolean drunkmoving(double target) {

        double ultraValue = 0;
        double gyroValue = getCurrentAngle() - (Math.round(getCurrentAngle() / 90) * 90);
        double spin = Math.max(minGyro, Math.min(maxGyro, gyroValue * gyroGain));

        if (ultraEnabled)
            ultraValue = Math.max(minUltra, Math.min(maxUltra, (ultra.cmUltrasonic() - target) * ultraGain));
        int fLeftMotorPower = (fLeftMotorTarget - fLeftMotor.getCurrentPosition());
        int fRightMotorPower = (fRightMotorTarget - fRightMotor.getCurrentPosition());
        int bLeftMotorPower = (bLeftMotorTarget - bLeftMotor.getCurrentPosition());
        int bRightMotorPower = (bRightMotorTarget - bRightMotor.getCurrentPosition());
        double max = Math.max(Math.max(Math.abs(fLeftMotorPower), Math.abs(bLeftMotorPower)), Math.max(Math.abs(fRightMotorPower), Math.abs(bRightMotorPower)));
        fLeftMotorPower += (ultraValue + spin) * max;
        fRightMotorPower += (ultraValue - spin) * max;
        bLeftMotorPower += (ultraValue + spin) * max;
        bRightMotorPower += (ultraValue - spin) * max;
        return Math.abs(fLeftMotorPower) < bettermovedeadzone && Math.abs(fRightMotorPower) < bettermovedeadzone && Math.abs(bLeftMotorPower) < bettermovedeadzone && Math.abs(bRightMotorPower) < bettermovedeadzone;
    }

    /***
     * check motor encoder positions relative to targets
     *
     * @return true if motors are close enough to the targets
     */
    public boolean gyromoving() {
        double maxGyro = 1;
        double minGyro = -maxGyro;

        double ultraValue = 0;
        double gyroValue = getCurrentAngle() - (Math.round(getCurrentAngle() / 45) * 45);
        double spin = Math.max(minGyro, Math.min(maxGyro, gyroValue * gyroGain));

        int fLeftMotorPower = (fLeftMotorTarget - fLeftMotor.getCurrentPosition());
        int fRightMotorPower = (fRightMotorTarget - fRightMotor.getCurrentPosition());
        int bLeftMotorPower = (bLeftMotorTarget - bLeftMotor.getCurrentPosition());
        int bRightMotorPower = (bRightMotorTarget - bRightMotor.getCurrentPosition());
        double max = Math.max(Math.max(Math.abs(fLeftMotorPower), Math.abs(bLeftMotorPower)), Math.max(Math.abs(fRightMotorPower), Math.abs(bRightMotorPower)));
        fLeftMotorPower += (ultraValue + spin) * max;
        fRightMotorPower += (ultraValue - spin) * max;
        bLeftMotorPower += (ultraValue + spin) * max;
        bRightMotorPower += (ultraValue - spin) * max;
        return Math.abs(fLeftMotorPower) < bettermovedeadzone && Math.abs(fRightMotorPower) < bettermovedeadzone && Math.abs(bLeftMotorPower) < bettermovedeadzone && Math.abs(bRightMotorPower) < bettermovedeadzone;
    }

    /***
     * for debugging bettermove
     *
     * @return (fLeftMotorPower, fRightMotorPower, bLeftMotorPower, bRightMotorPower)
     */
    public String getErrors() {
        int fLeftMotorPower = (fLeftMotorTarget - fLeftMotor.getCurrentPosition());
        int fRightMotorPower = (fRightMotorTarget - fRightMotor.getCurrentPosition());
        int bLeftMotorPower = (bLeftMotorTarget - bLeftMotor.getCurrentPosition());
        int bRightMotorPower = (bRightMotorTarget - bRightMotor.getCurrentPosition());
        return String.format(Locale.ENGLISH, "%d, %d, %d, %d", fLeftMotorPower, fRightMotorPower, bLeftMotorPower, bRightMotorPower);
    }

}

