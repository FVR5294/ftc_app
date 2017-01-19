package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.Range;

import static org.firstinspires.ftc.teamcode.robotconfig.dl;
/**
 * Created by Matthew Hotham on 11/5/2016.
 */
//@TeleOp(name = "MattTeleOP", group = "2016")

public class MatTeleOP extends OpMode {

    private final static double buttonPusher_MIN_RANGE = 0.40;
    private final static double buttonPusher_MAX_RANGE = 0.80;

    private final static double Tilt_MAX_RANGE = 1.00;
    private final static double Tilt_MIN_RANGE = 0.00;

    private final static double capRight_MAX_RANGE = 1.00;
    private final static double capRight_MIN_RANGE = 0.00;

    private final static double capLeft_MAX_RANGE = 1.00;
    private final static double capLeft_MIN_RANGE = 0.00;

    robotconfig robot = new robotconfig();

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
    private double cam;

    @Override
    public void init() {

        robot.init(this);
        robot.move(0, 0, 0);
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");
        updateTelemetry(telemetry);
        robot.pushButton(0);
        tiltPosition = 0.50;
        capLeftPosition = 0.00;
        capRightPosition = 0.00;

    }

    @Override
    public void loop() {

        forward = -gamepad1.left_stick_y;
        right = gamepad1.left_stick_x;
        spin = gamepad1.right_stick_x;

        if (gamepad1.right_bumper || gamepad1.left_bumper) {
            forward *= -1;
            right *= -1;
        }

        forward = Range.clip(forward, -1, 1);
        right = Range.clip(right, -1, 1);
        spin = Range.clip(spin, -1, 1);
        spinner = Range.clip(spinner, -1, 1);
        reeler = Range.clip(reeler, -1, 1);
        vexes = Range.clip(vexes, -1, 1);
        cam = Range.clip(cam, -1, 1);

        robot.move(forward, right, spin);

        robot.spinner.setPower(spinner);
        robot.reeler.setPower(reeler);


        if (gamepad1.dpad_left || gamepad1.x) {
            robot.pushSoftButton(-1);
        }

        if (gamepad1.dpad_right || gamepad1.b) {
            robot.pushSoftButton(1);
        }

        if (gamepad2.a) {
            spinner = -gamepad2.left_stick_y;
        }

        if (gamepad2.b) {
            vexes = -gamepad2.left_stick_y * 0.5 + 0.5;
                robot.lvex.setPosition(vexes);
                robot.rvex.setPosition(vexes);
        }

        if (gamepad2.right_bumper) {
            try {
                robot.puncher.setPower(1);
            } catch (Exception err) {
                robotconfig.addlog(dl, "error", "failed to set power of puncher");
            }
        } else {
            try {
                robot.puncher.setPower(0);
            } catch (Exception err) {
                robotconfig.addlog(dl, "error", "failed to set power of puncher");
            }
        }

        reeler = -gamepad2.right_stick_y;

        if (gamepad2.dpad_up) {
            capLeftPosition = 0.66;
            tiltPosition = 1.00;
        }

        if (gamepad2.dpad_right) {
            capLeftPosition = 0.90;
            tiltPosition = 1.00;
        }

        if (gamepad2.dpad_down) {
            capLeftPosition = 0.90;
            tiltPosition = 0.00;
        }

        if (gamepad2.dpad_left) {
            capLeftPosition = 0.66;
            tiltPosition = 0.00;
        }

        if (gamepad2.left_bumper) {
            capLeftPosition = 0;
            tiltPosition = 0.50;
        }

        tiltPosition = Range.clip(tiltPosition, Tilt_MIN_RANGE, Tilt_MAX_RANGE);
        capRightPosition = Range.clip(capRightPosition, capRight_MIN_RANGE, capRight_MAX_RANGE);
        capLeftPosition = Range.clip(capLeftPosition, capLeft_MIN_RANGE, capLeft_MAX_RANGE);

        robot.tilt.setPosition(tiltPosition);
        robot.capLeft.setPosition(capLeftPosition);
        robot.capRight.setPosition(capLeftPosition);

        telemetry.addData("Forward", "%.2f", forward);
        telemetry.addData("Right", "%.2f", right);
        telemetry.addData("Spin", "%.2f", spin);
        telemetry.addData("Tilt", "%.2f", tiltPosition);
        telemetry.addData("capLeft", "%.2f", capLeftPosition);
        telemetry.addData("vexes", "%.2f", vexes);
    }

}
