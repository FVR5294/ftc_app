package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Matthew Hotham on 11/5/2016.
 */

@TeleOp(name = "TeleOp_Revised", group = "2016")

public class TeleOp_Revised extends OpMode {

    // TODO: 1/12/2017 Fine tune button pusher range
    private static double buttonPusher_MIN_RANGE = 0.35;
    private static double buttonPusher_MAX_RANGE = 0.75;

    private static double Tilt_MAX_RANGE = 1.00;
    private static double Tilt_MIN_RANGE = 0.00;

    private static double capRight_MAX_RANGE = 1.00;
    private static double capRight_MIN_RANGE = 0.00;

    private static double capLeft_MAX_RANGE = 1.00;
    private static double capLeft_MIN_RANGE = 0.00;
    private static double buttonPusherDelta = 0.02;
    private static double tiltDelta = 0.02;
    private static double capLeftDelta = 0.01;
    private static double capRightDelta = 0.01;
    private static int pulses = 2240;
    robotconfig robot = new robotconfig();
    private double buttonPusherPosition = 0;
    private double tiltPosition = 0;
    private double capLeftPosition = 0;
    private double capRightPosition = 0;
    private double forward = 0;
    private double right = 0;
    private double spin = 0;
    private double spinner = 0;
    private double reeler = 0;
    private double vexes;
    private double puncher;
    private boolean limitPuncher;
    private int endpulses = 0;

    private boolean previousAState = false;
    private boolean previousGaryState = false;
    private boolean spinnerState = false;
    private boolean puncherState = false;


    @Override
    public void init() {

        robot.init(this);
        robot.move(0, 0, 0);
        robot.disableMotorEncoders();
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");
        updateTelemetry(telemetry);

        buttonPusherPosition = 0.5;
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

        robot.move(forward, right, spin);

        robot.reeler.setPower(reeler);

        if (gamepad2.right_bumper) {
            vexes = -gamepad2.left_stick_y * 0.5 + 0.5;
            robot.rvex.setPosition(vexes);
            robot.lvex.setPosition(vexes);
        }

        if (gamepad2.left_bumper) {
            capLeftPosition = 1;
            robot.capLeft.setPosition(capLeftPosition);
            capRightPosition = 1;
            robot.capRight.setPosition(capRightPosition);
            tiltPosition = 1;
            robot.tilt.setPosition(tiltPosition);
        }

        if (gamepad1.a) {
            robot.puncher.setPower(1);
            puncherState = true;
        }

        if (puncherState) {
            if (!robot.garry.isPressed() && previousGaryState) {
                robot.puncher.setPower(0);
                puncherState = false;
            }
        }

        previousGaryState = robot.garry.isPressed();

        if (gamepad1.dpad_left) {
            buttonPusherPosition -= buttonPusherDelta;
            buttonPusherPosition = Range.clip(buttonPusherPosition, buttonPusher_MIN_RANGE, buttonPusher_MAX_RANGE);
            robot.buttonPusher.setPosition(buttonPusherPosition);
        } else if (gamepad1.dpad_right) {
            buttonPusherPosition += buttonPusherDelta;
            buttonPusherPosition = Range.clip(buttonPusherPosition, buttonPusher_MIN_RANGE, buttonPusher_MAX_RANGE);
            robot.buttonPusher.setPosition(buttonPusherPosition);
        }

        if (gamepad1.left_bumper) {
            if (!previousAState) {
                if (spinnerState) {
                    spinnerState = false;
                    robot.spinner.setPower(0);
                } else {
                    spinnerState = true;
                    robot.spinner.setPower(1);
                }
                previousAState = true;
            }
        } else {
            previousAState = false;
        }

        if (gamepad2.dpad_right) {
            capLeftPosition += capLeftDelta;
            capLeftPosition = Range.clip(capLeftPosition, capLeft_MIN_RANGE, capLeft_MAX_RANGE);
            robot.capLeft.setPosition(capLeftPosition);
        } else if (gamepad2.dpad_left) {
            capLeftPosition -= capLeftDelta;
            capLeftPosition = Range.clip(capLeftPosition, capLeft_MIN_RANGE, capLeft_MAX_RANGE);
            robot.capLeft.setPosition(capLeftPosition);
        }

        if (gamepad2.x) {
            capRightPosition += capRightDelta;
            capRightPosition = Range.clip(capRightPosition, capRight_MIN_RANGE, capRight_MAX_RANGE);
            robot.capRight.setPosition(capRightPosition);
        } else if (gamepad2.b) {
            capRightPosition -= capRightDelta;
            capRightPosition = Range.clip(capRightPosition, capRight_MIN_RANGE, capRight_MAX_RANGE);
            robot.capRight.setPosition(capRightPosition);
        }

        if (gamepad2.a) {
            tiltPosition -= tiltDelta;
            tiltPosition = Range.clip(tiltPosition, Tilt_MIN_RANGE, Tilt_MAX_RANGE);
            robot.tilt.setPosition(tiltPosition);
        } else if (gamepad2.y) {
            tiltPosition += tiltDelta;
            tiltPosition = Range.clip(tiltPosition, Tilt_MIN_RANGE, Tilt_MAX_RANGE);
            robot.tilt.setPosition(tiltPosition);
        }

        telemetry.addData("Forward", "%.2f", forward);
        telemetry.addData("Right", "%.2f", right);
        telemetry.addData("Spin", "%.2f", spin);
        telemetry.addData("ButtonPusher", "%.2f", buttonPusherPosition);
        telemetry.addData("Tilt", "%.2f", tiltPosition);
        telemetry.addData("capLeft", "%.2f", capLeftPosition);
        telemetry.addData("vexes", "%.2f", vexes);
        telemetry.addData("puncher", "%.2f", puncher);
        telemetry.addData("garry", "%b", !robot.garry.isPressed());

    }

}