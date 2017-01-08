package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import static java.lang.Thread.sleep;
import static org.firstinspires.ftc.teamcode.robotconfig.dl;

/**
 * Created by Matthew Hotham on 11/5/2016.
 */

@TeleOp(name = "TeleOp_Revised", group = "2016")

public class TeleOp_Revised extends OpMode {

    private final static double buttonPusher_MIN_RANGE = 0.40;
    private final static double buttonPusher_MAX_RANGE = 0.80;

    private final static double Tilt_MAX_RANGE = 1.00;
    private final static double Tilt_MIN_RANGE = 0.00;

    private final static double capRight_MAX_RANGE = 1.00;
    private final static double capRight_MIN_RANGE = 0.00;

    private final static double capLeft_MAX_RANGE = 1.00;
    private final static double capLeft_MIN_RANGE = 0.00;

    robotconfig robot = new robotconfig();

    private double buttonPusherPosition;
    private double tiltPosition;
    private double capLeftPosition;
    private double capRightPosition;

    private double buttonPusherDelta = 0.02;
    private double tiltDelta = 0.02;
    private double capLeftDelta = 0.01;
    private double capRightDelta = 0.01;

    private double forward = 0;
    private double right = 0;
    private double spin = 0;
    private double spinner = 0;
    private double reeler = 0;

    private double vexes;
    private double puncher;
    private boolean limitPuncher;
    int pulses = 2240;
    int endpulses = 0;

    private boolean previousAState = false;
    private boolean spinnerState = false;

    @Override
    public void init() {

        robot.init(this);
        robot.move(0, 0, 0);
        robot.disableMotorEncoders();
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");
        updateTelemetry(telemetry);

        buttonPusherPosition = 0.56;
        tiltPosition = 0.50;
        capLeftPosition = 0.00;
        capRightPosition = 0.00;

    }

    @Override
    public void loop() {

        forward = -gamepad1.left_stick_y * Math.abs(gamepad1.left_stick_y);
        right = gamepad1.left_stick_x * Math.abs(gamepad1.left_stick_x);
        spin = gamepad1.right_stick_x * Math.abs(gamepad1.right_stick_x);
        reeler = -gamepad2.right_stick_y;

        if (gamepad1.right_bumper) {
            forward *= -1;
            right *= -1;
        }

        forward = Range.clip(forward, -1, 1);
        right = Range.clip(right, -1, 1);
        spin = Range.clip(spin, -1, 1);
        spinner = Range.clip(spinner, -1, 1);
        reeler = Range.clip(reeler, -1, 1);
        vexes = Range.clip(vexes, -1, 1);
        puncher = Range.clip(puncher, -1, 1);

        robot.move(forward, right, spin);

        robot.reeler.setPower(reeler);

        if (gamepad2.right_bumper) {
            vexes = -gamepad2.left_stick_y * 0.5 + 0.5;
        }

        if (gamepad1.y) {
                robot.puncher.setPower(1);
                endpulses = robot.puncher.getCurrentPosition() + pulses;
        }

        if (gamepad1.dpad_left) {
            buttonPusherPosition -= buttonPusherDelta;
        }

        if (gamepad1.dpad_right) {
            buttonPusherPosition += buttonPusherDelta;
        }

        if (gamepad1.a && !previousAState) {
            if (spinnerState) {
                spinnerState = false;
                robot.spinner.setPower(0);
            } else {
                spinnerState = true;
                robot.spinner.setPower(1);
            }
        }

        previousAState = gamepad1.a;

        if (gamepad2.dpad_right) {
            capLeftPosition += capLeftDelta;
        }

        if (gamepad2.dpad_left) {
            capLeftPosition -= capLeftDelta;
        }

        if (gamepad2.x) {
            capRightPosition += capRightDelta;
        }

        if (gamepad2.b) {
            capRightPosition -= capRightDelta;
        }

        if (gamepad2.a) {
            tiltPosition -= tiltDelta;
        }

        if (gamepad2.y) {
            tiltPosition += tiltDelta;
        }

        buttonPusherPosition = Range.clip(buttonPusherPosition, buttonPusher_MIN_RANGE, buttonPusher_MAX_RANGE);
        tiltPosition = Range.clip(tiltPosition, Tilt_MIN_RANGE, Tilt_MAX_RANGE);
        capRightPosition = Range.clip(capRightPosition, capRight_MIN_RANGE, capRight_MAX_RANGE);
        capLeftPosition = Range.clip(capLeftPosition, capLeft_MIN_RANGE, capLeft_MAX_RANGE);

        robot.buttonPusher.setPosition(buttonPusherPosition);
        robot.tilt.setPosition(tiltPosition);
        robot.capLeft.setPosition(capLeftPosition);
        robot.capRight.setPosition(capRightPosition);

        telemetry.addData("Forward", "%.2f", forward);
        telemetry.addData("Right", "%.2f", right);
        telemetry.addData("Spin", "%.2f", spin);
        telemetry.addData("ButtonPusher", "%.2f", buttonPusherPosition);
        telemetry.addData("Tilt", "%.2f", tiltPosition);
        telemetry.addData("capLeft", "%.2f", capLeftPosition);
        telemetry.addData("vexes", "%.2f", vexes);
        telemetry.addData("puncher", "%.2f", puncher);

    }

}