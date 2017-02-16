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

    private static double buttonPusher_MIN_RANGE = 0.35;
    private static double buttonPusher_MAX_RANGE = 0.65;

    private static double Tilt_MAX_RANGE = 0.95;
    private static double Tilt_MIN_RANGE = 0.05;

    private static double capRight_MAX_RANGE = 0.95;
    private static double capRight_MIN_RANGE = 0.05;

    private static double capLeft_MAX_RANGE = 0.95;
    private static double capLeft_MIN_RANGE = 0.05;
    private static double buttonPusherDelta = 0.02;
    private static double tiltDelta = 0.02;
    private static double capLeftDelta = 0.02;
    private static double capRightDelta = 0.02;
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
    private boolean speedToggleFlag = false;
    private boolean slowState = false;

    private ElapsedTime loopTimer = new ElapsedTime();
    private double previousTimes[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    private int timerStage = 0;

    @Override
    public void init() {

        robot.init(this);
        robot.move(0, 0, 0);
        robot.disableMotorEncoders();
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");
        updateTelemetry(telemetry);
        robot.capLeft.getController().pwmDisable();
        buttonPusherPosition = 0.5;
        tiltPosition = 0.95;
        capLeftPosition = 0.7;
        capRightPosition = 0.7;
        loopTimer.reset();

    }

    @Override
    public void loop() {

        forward = -gamepad1.left_stick_y * Math.abs(gamepad1.left_stick_y);
        right = gamepad1.left_stick_x * Math.abs(gamepad1.left_stick_x);
        spin = gamepad1.right_stick_x * Math.abs(gamepad1.right_stick_x);
        reeler = -gamepad2.right_stick_y;

        if (slowState) {
            forward = forward * 0.25;
            right = right * 0.60;
            spin = spin * 0.35;
        }

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


//        if (gamepad1.right_bumper) {
//            forward *= -1;
//            right *= -1;
//        }


        robot.move(forward, right, spin);

        robot.reeler.setPower(reeler);

        if (gamepad2.right_bumper) {
            vexes = -gamepad2.left_stick_y * 0.5 + 0.5;
            robot.rvex.setPosition(vexes);
            robot.lvex.setPosition(vexes);
        }

        if (gamepad2.left_bumper) {
            robot.capLeft.getController().pwmEnable();
            robot.capLeft.setPosition(capLeftPosition);
            robot.capRight.setPosition(capRightPosition);
            robot.tilt.setPosition(tiltPosition);
        }

        if (gamepad1.y) {
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

        if (gamepad2.back)
            robot.capLeft.getController().pwmDisable();
        else if (gamepad2.left_stick_button)
            robot.capLeft.getController().pwmEnable();

        telemetry.addData("Forward", "%.2f", forward);
        telemetry.addData("Right", "%.2f", right);
        telemetry.addData("Spin", "%.2f", spin);
        telemetry.addData("ButtonPusher", "%.2f", buttonPusherPosition);
        telemetry.addData("Tilt", "%.2f", tiltPosition);
        telemetry.addData("capLeft", "%.2f", capLeftPosition);
        telemetry.addData("vexes", "%.2f", vexes);
        telemetry.addData("puncher", "%.2f", puncher);
        telemetry.addData("garry", "%b", !robot.garry.isPressed());

        telemetry.addData("1", "%d", previousTimes[timerStage % 12]);
        telemetry.addData("2", "%d", previousTimes[1 + timerStage % 12]);
        telemetry.addData("3", "%d", previousTimes[2 + timerStage % 12]);
        telemetry.addData("4", "%d", previousTimes[3 + timerStage % 12]);
        telemetry.addData("5", "%d", previousTimes[4 + timerStage % 12]);
        telemetry.addData("6", "%d", previousTimes[5 + timerStage % 12]);
        telemetry.addData("7", "%d", previousTimes[6 + timerStage % 12]);
        telemetry.addData("8", "%d", previousTimes[7 + timerStage % 12]);
        telemetry.addData("9", "%d", previousTimes[8 + timerStage % 12]);
        telemetry.addData("10", "%d", previousTimes[9 + timerStage % 12]);
        telemetry.addData("11", "%d", previousTimes[10 + timerStage % 12]);
        telemetry.addData("12", "%d", previousTimes[11 + timerStage % 12]);
        telemetry.addData("13", "%d", previousTimes[12 + timerStage % 12]);

        loopTimer.reset();
        timerStage++;
        timerStage = timerStage % 12;
    }

//        private String graph(int x) {
//        switch(x) {
//            case 0:
//                return "";
//            case 1:
//                return ".";
//            case 2:
//                return "..";
//            case 3:
//                return "...";
//            case 4:
//                return "....";
//            case 5:
//                return ".... .";
//            case 6:
//                return ".... ..";
//            case 7:
//                return ".... ...";
//            case 8:
//                return ".... ....";
//            case 9:
//                return ".... .... .";
//            case 10:
//                return ".... .... ..";
//            case 11:
//                return ".... .... ...";
//            case 12:
//                return ".... .... ....";
//            case 13:
//                return ".... .... .... .";
//            case 14:
//                return ".... .... .... ..";
//            case 15:
//                return ".... .... .... ...";
//            case 16:
//                return ".... .... .... ....";
//            case 17:
//                return ".... .... .... .... .";
//            case 18:
//                return ".... .... .... .... ..";
//            case 19:
//                return ".... .... .... .... ...";
//            default:
//                return ".... .... .... .... ....";
//        }
//    }

}