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
    private final double pulsesReduced = pulses * 0.9;
    private final double rampNumb = 3.5 / pulses;
    robotconfig robot = new robotconfig();
    private double endpulses = 0.0;
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
    private boolean previousGaryState = false;
    private boolean spinnerState = false;
    private boolean puncherState = false;
    private boolean speedToggleFlag = false;
    private boolean slowState = false;

    private ElapsedTime loopTimer = new ElapsedTime();

    @Override
    public void init() {

        robot.init(this);
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

        /*
         *  This added code is to remove power from the cap ball grabber servos
         *  to be used if we need to disengage the arms and allow the to move
         *  freely.
         */

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

        vexes = -gamepad2.left_stick_y * 0.5 + 0.5 + gamepad1.right_trigger / 2 + gamepad2.right_trigger / 2 - gamepad2.left_trigger / 2;
        robot.rvex.setPosition(vexes);
        robot.lvex.setPosition(vexes);

        if (gamepad2.right_bumper)
            robot.theHammerOfDawn.setPosition(1);
        else
            robot.theHammerOfDawn.setPosition(vexes);


        if (gamepad1.left_bumper || gamepad2.left_bumper) {
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

        if (gamepad1.left_trigger > 0.1 || gamepad1.right_trigger > 0.1) {
            robot.spinner.setPower(gamepad1.right_trigger - gamepad1.left_trigger);
        } else {
            if (spinnerState) {
                robot.spinner.setPower(1);
            } else {
                robot.spinner.setPower(0);
            }
        }

        if (puncherState) {
            if (!robot.garry.isPressed() && previousGaryState) {
                robot.puncher.setPower(0);
                puncherState = false;
            } else {
                puncher = Math.min(1, Math.max(0.6, (endpulses - robot.puncher.getCurrentPosition()) * rampNumb));
                robot.puncher.setPower(puncher);
            }
        }

        if (gamepad1.y && !puncherState) {
            robot.puncher.setPower(1);
            puncherState = true;
//            if (!robot.garry.isPressed())
            endpulses = pulses + robot.puncher.getCurrentPosition();
//            else
//                endpulses = pulsesReduced + robot.puncher.getCurrentPosition();
        }

        reeler = -gamepad2.right_stick_y;

        robot.reeler.setPower(reeler);

//        if (gamepad2.left_bumper) {
//            capLeftPosition = 0.7;
//            robot.capLeft.setPosition(capLeftPosition);
//            capRightPosition = 0.7;
//            robot.capRight.setPosition(capRightPosition);
//            tiltPosition = 0.95;
//            robot.tilt.setPosition(tiltPosition);
//        }

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
//        else if (gamepad2.left_stick_button)
//            robot.capLeft.getController().pwmEnable();


        previousGaryState = robot.garry.isPressed();

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