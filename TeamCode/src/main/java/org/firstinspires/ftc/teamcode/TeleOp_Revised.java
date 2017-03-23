package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import static org.firstinspires.ftc.teamcode.robotconfig.dl;

/**
 * the main TeleOp program for our robot
 */

@TeleOp(name = "TeleOp_Revised", group = "2016")

public class TeleOp_Revised extends OpMode {

    private final double buttonPusher_MIN_RANGE = 0.35;
    private final double buttonPusher_MAX_RANGE = 0.65;

    private final double Tilt_MAX_RANGE = 0.95;
    private final double Tilt_MIN_RANGE = 0.05;

    private final double capRight_MAX_RANGE = 0.95;
    private final double capRight_MIN_RANGE = 0.05;

    private final double capLeft_MAX_RANGE = 0.95;
    private final double capLeft_MIN_RANGE = 0.05;
    private final double buttonPusherDelta = 0.02;
    private final double tiltDelta = 0.02;
    private final double capLeftDelta = 0.02;
    private final double capRightDelta = 0.02;
    private final double pulses = 2240.0;
    private final double rampNumb = 3.5 / pulses;
    robotconfig robot = new robotconfig();
    private double pulsesOffset = 0.0;

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
    private double puncher = 0;
    private boolean limitPuncher;

    private boolean previousAState = false;
    private boolean puncherIsActive = false;
    private boolean spinnerState = false;
    private boolean puncherState = false;
    private boolean speedToggleFlag = false;
    private boolean slowState = false;

    private ElapsedTime loopTimer = new ElapsedTime();

    @Override
    public void init() {

        robot.init(this);
        pulsesOffset = (robot.puncher.getCurrentPosition() % pulses);
        robot.move(0, 0, 0);
//        robot.disableMotorEncoders();
        // Send telemetry message to signify robot waiting;
//        robot.capLeft.getController().pwmDisable();
        telemetry.addData("Say", "Hello Driver");
        updateTelemetry(telemetry);

        buttonPusherPosition = 0.5;
        tiltPosition = 0.50;
        capLeftPosition = 0.05;
        capRightPosition = 0.05;
        loopTimer.reset();

    }

    @Override
    public void loop() {

        forward = -gamepad1.left_stick_y * Math.abs(gamepad1.left_stick_y);
        right = gamepad1.left_stick_x * Math.abs(gamepad1.left_stick_x);
        spin = gamepad1.right_stick_x * Math.abs(gamepad1.right_stick_x);

        if (slowState) {
            forward = forward * 0.25;
            right = right * 0.60;
            spin = spin * 0.35;
        }

        robot.move(forward, right, spin);

        robotconfig.addlog(dl, "in TeleOp_Revised", "top of main loop");

        if (gamepad1.right_bumper) {          // bumper is down
            if (speedToggleFlag) {   // was down last time, so ignore
                // this time
            } else {                                // ok, this the first time
                // through on a new buper down
                speedToggleFlag = true;

                slowState = !slowState;

            }

        } else {
            speedToggleFlag = false;     // trigger is not pulled, so reset
            // trigger seen flag
        }

        vexes = -gamepad2.left_stick_y * 0.5 + 0.5 + gamepad2.right_trigger / 2 + gamepad1.right_trigger / 2 - gamepad2.left_trigger / 2;
        robot.rvex.setPosition(vexes);
        robot.lvex.setPosition(vexes);
        robot.theHammerOfDawn.setPosition(vexes);

        if (gamepad1.left_bumper || gamepad2.left_bumper) {
            if (!previousAState) {
                spinnerState = !spinnerState;
                previousAState = true;
            }
        } else {
            previousAState = false;
        }

        if (gamepad1.left_trigger > 0.1 || gamepad1.right_trigger > 0.1) {
            robot.spinner.setPower(gamepad1.right_trigger - gamepad1.left_trigger);
        } else {
            if (spinnerState) {
                robot.spinner.setPower(1);
            } else {
                robot.spinner.setPower(0);
            }
        }

        //puncherState is true if the puncher is on but in the beginning stage where the limit switch is still pressed
        if (puncherState && !robot.garry.isPressed()) {

            //signal that puncher got past initial stage
            puncherState = false;//without this line, the end is the same as the beginning to the limit switch

            pulsesOffset = (robot.puncher.getCurrentPosition() % pulses);

        } else if (robot.garry.isPressed()) {//always activate puncher if up

            puncher = Math.min(1, Math.max((pulses - ((robot.puncher.getCurrentPosition() - pulsesOffset) % pulses)) * rampNumb, 0.6));

            robot.puncher.setPower(puncher);

        } else if (puncherIsActive) {
            //puncher stop code is only run per puncher activation

            puncher = 0;
            robot.puncher.setPower(puncher);
            puncherIsActive = false;
        }

        if (gamepad1.y && !puncherIsActive) {
            //fire
            puncher = 1;
            robot.puncher.setPower(puncher);
            puncherIsActive = true;
            puncherState = true;
        }

        reeler = -gamepad2.right_stick_y;

        robot.reeler.setPower(reeler);

        if (gamepad1.dpad_left) {
            buttonPusherPosition -= buttonPusherDelta;
            buttonPusherPosition = Range.clip(buttonPusherPosition, buttonPusher_MIN_RANGE, buttonPusher_MAX_RANGE);
            robot.buttonPusher.setPosition(buttonPusherPosition);
        } else if (gamepad1.dpad_right) {
            buttonPusherPosition += buttonPusherDelta;
            buttonPusherPosition = Range.clip(buttonPusherPosition, buttonPusher_MIN_RANGE, buttonPusher_MAX_RANGE);
            robot.buttonPusher.setPosition(buttonPusherPosition);
        }

        if (gamepad2.dpad_right) {
            capLeftPosition += Math.max(capLeftDelta, 0.1 * (1 - capLeftPosition));
            capLeftPosition = Range.clip(capLeftPosition, capLeft_MIN_RANGE, capLeft_MAX_RANGE);
            robot.capLeft.setPosition(capLeftPosition);
        } else if (gamepad2.dpad_left) {
            capLeftPosition -= capLeftDelta;
            capLeftPosition = Range.clip(capLeftPosition, capLeft_MIN_RANGE, capLeft_MAX_RANGE);
            robot.capLeft.setPosition(capLeftPosition);
        }

        if (gamepad2.x) {
            capRightPosition += Math.max(capRightDelta, 0.1 * (1 - capRightPosition));
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

        if (gamepad2.back)
            robot.capLeft.getController().pwmDisable();

        telemetry.addData("latency", "%.2f", loopTimer.milliseconds());
        loopTimer.reset();
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

    @Override
    public void stop() {
        robot.theHammerOfDawn.setPosition(0.5);
        robot.rvex.setPosition(0.5);
        robot.lvex.setPosition(0.5);
    }

}