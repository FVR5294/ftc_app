package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Func;

import java.util.Locale;

import static org.firstinspires.ftc.teamcode.measurements.x1;
import static org.firstinspires.ftc.teamcode.measurements.x3;
import static org.firstinspires.ftc.teamcode.measurements.y1;
import static org.firstinspires.ftc.teamcode.measurements.y3;
import static org.firstinspires.ftc.teamcode.robotconfig.dl;

/**
 * the main TeleOp program for our robot
 */

@TeleOp(name = "TeleOp_Revised")

public class TeleOp_Revised extends OpMode {

    final double buttonPusher_MIN_RANGE = 0.35;
    final double buttonPusher_MAX_RANGE = 0.65;

    final double Tilt_MAX_RANGE = x3;
    final double Tilt_MIN_RANGE = x1;

    final double capRight_MAX_RANGE = 0.95;
    final double capRight_MIN_RANGE = 0.05;

    final double capLeft_MAX_RANGE = 0.95;
    final double capLeft_MIN_RANGE = 0.05;
    final double buttonPusherDelta = 0.02;
    final double tiltDelta = 0.02;
    final double capLeftDelta = 0.02;
    final double capRightDelta = 0.02;

    final double pulses = 2240.0;
    final double rampNumb = 3.5 / pulses;

    robotconfig robot = new robotconfig();
    double endpulses = 0.0;
    double buttonPusherPosition = 0;
    double tiltPosition = 1.0 - 140.0 / 255.0;
    double tilt2Position = 0;
    double capLeftPosition = 0;
    double capRightPosition = 0;
    double forward = 0;
    double right = 0;
    double spin = 0;
    double spinner = 0;
    double reeler = 0;
    double vexes = 0.5;
    double puncher = 0;
    int minutes = 0;
    int seconds = 0;
    ElapsedTime matchTimer = new ElapsedTime();
    boolean puncherBroken = false;
    //    boolean previous2state = false;
//    boolean previous1state = false;
    boolean previousAState = false;
    boolean previousGaryState = false;
    boolean spinnerState = false;
    boolean puncherState = false;
    boolean speedToggleFlag = false;
    boolean slowState = false;
    ElapsedTime loopTimer = new ElapsedTime();
    double loopTime = 0;
    double totalLoopTime = 0;
    double loopCount = 1;
    double maxLoopTime = 0;

    @Override
    public void init() {

//        robotconfig.debugMode = true;
        robot.init(this);

        robot.disableMotorEncoders();
        // Send telemetry message to signify robot waiting;
//        robot.capLeft.getController().pwmDisable();
//        robot.enableMotorBreak();
        telemetry.update();
        buttonPusherPosition = 0.5;
        capLeftPosition = 0.05;
        capRightPosition = 0.05;
        loopTimer.reset();
        activateTelemetry();
        /////////////////////////Ben... you may want to disable some telemetry by commenting out the line before or after this but I highly doubt it will make a measurable difference in performance due to how efficient this is and I will pay you a dollar for reading this very long comment if you send me a text message with an approximation of the number PI
        activateDebugTelemetry();
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        matchTimer.reset();
        loopTimer.reset();
    }

    @Override
    public void loop() {

        loopTime = loopTimer.milliseconds();
        loopTimer.reset();
        totalLoopTime += loopTime;
        loopCount++;
        maxLoopTime = Math.max(maxLoopTime, loopTime);

        forward = -gamepad1.left_stick_y;
        right = gamepad1.left_stick_x;
        spin = gamepad1.right_stick_x;

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

        if (spinnerState)
            spinner = 1;
        else
            spinner = 0;

        vexes = -gamepad2.left_stick_y / 2 + 0.5 + gamepad2.right_trigger / 2 - gamepad2.left_trigger / 2;

        robot.rvex.setPosition(vexes);
        robot.lvex.setPosition(vexes);

        if (gamepad1.left_trigger > 0.1 || gamepad1.right_trigger > 0.1)
            spinner = gamepad1.right_trigger - gamepad1.left_trigger;

        robot.spinner.setPower(spinner);

        if (vexes > 0.55)
            robot.theHammerOfDawn.setPosition(1);
        else if (vexes < 0.45)
            robot.theHammerOfDawn.setPosition(0);
        else
            robot.theHammerOfDawn.setPosition(vexes);

        if (gamepad1.left_bumper || gamepad2.left_bumper) {
            if (!previousAState) {
                spinnerState = !spinnerState;
                previousAState = true;
            }
        } else {
            previousAState = false;
        }

        if (puncherState) {
            if ((!robot.garry.isPressed() && previousGaryState) || (puncherBroken && endpulses - robot.puncher.getCurrentPosition() < -100)) {
                robot.puncher.setPower(0);
                puncherState = false;
            } else {
                puncher = Math.min(1, Math.max(0.6, (endpulses - robot.puncher.getCurrentPosition()) * rampNumb));
                robot.puncher.setPower(puncher);
            }
        }

        if (gamepad1.y && !puncherState) {
//            if (!robot.garry.isPressed())
            robot.puncher.setPower(1);
            puncherState = true;
//            if (!robot.garry.isPressed())
            endpulses = pulses + robot.puncher.getCurrentPosition();
//            else
//                endpulses = pulsesReduced + robot.puncher.getCurrentPosition();
        }

        previousGaryState = robot.garry.isPressed();

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
            tilt2Position = (tiltPosition - x1) * (y3 - y1) / (x3 - x1) + y1;
            robot.tilt.setPosition(tiltPosition);
            robot.tilt2.setPosition(tilt2Position);
        } else if (gamepad2.y) {
            tiltPosition += tiltDelta;
            tiltPosition = Range.clip(tiltPosition, Tilt_MIN_RANGE, Tilt_MAX_RANGE);
            tilt2Position = (tiltPosition - x1) * (y3 - y1) / (x3 - x1) + y1;
            robot.tilt.setPosition(tiltPosition);
            robot.tilt2.setPosition(tilt2Position);
        }

        //if anything fails...
        if (gamepad2.back) {
            robot.capLeft.getController().pwmDisable();
            robot.tilt.getController().pwmDisable();
            robot.buttonPusher.getController().pwmDisable();
            robot.autoIntake = false;
            robot.eject = false;
            puncherBroken = true;
        }

    }

    @Override
    public void stop() {
        robot.theHammerOfDawn.setPosition(0.5);
        robot.rvex.setPosition(0.5);
        robot.lvex.setPosition(0.5);
    }

    void activateTelemetry() {

        telemetry.addLine()
                .addData("match time", new Func<String>() {
                    @Override
                    public String value() {
                        return String.format(Locale.ENGLISH, "%d:%d", minutes, seconds);
                    }
                });

        telemetry.addLine()
                .addData("loop time", new Func<String>() {
                    @Override
                    public String value() {
                        return String.format(Locale.ENGLISH, "%.2f", loopTime);
                    }
                })
                .addData("avg", new Func<String>() {
                    @Override
                    public String value() {
                        return String.format(Locale.ENGLISH, "%.2f", totalLoopTime / loopCount);
                    }
                })
                .addData("max", new Func<String>() {
                    @Override
                    public String value() {
                        return String.format(Locale.ENGLISH, "%.2f", maxLoopTime);
                    }
                });
    }

    void activateDebugTelemetry() {

        telemetry.addLine()
                .addData("Forward", new Func<String>() {
                    @Override
                    public String value() {
                        return String.format(Locale.ENGLISH, "%.2f", forward);
                    }
                })
                .addData("Right", new Func<String>() {
                            @Override
                            public String value() {
                                return String.format(Locale.ENGLISH, "%.2f", right);
                            }
                        }
                )
                .addData("Spin", new Func<String>() {
                    @Override
                    public String value() {
                        return String.format(Locale.ENGLISH, "%.2f", spin);
                    }
                });
        telemetry.addLine()
                .addData("Puncher", new Func<String>() {
                            @Override
                            public String value() {
                                return String.format(Locale.ENGLISH, "%.2f", puncher);
                            }
                        }
                )
                .addData("garry", new Func<String>() {
                    @Override
                    public String value() {
                        return String.format(Locale.ENGLISH, "%b", !robot.garry.isPressed());
                    }
                })
                .addData("vexes", new Func<String>() {
                    @Override
                    public String value() {
                        return String.format(Locale.ENGLISH, "%.2f", vexes);
                    }
                });
        telemetry.addLine()
                .addData("Tilt", new Func<String>() {
                            @Override
                            public String value() {
                                return String.format(Locale.ENGLISH, "%.2f", tiltPosition);
                            }
                        }
                )
                .addData("Tilt2", new Func<String>() {
                            @Override
                            public String value() {
                                return String.format(Locale.ENGLISH, "%.2f", tilt2Position);
                            }
                        }
                )
                .addData("capLeft", new Func<String>() {
                    @Override
                    public String value() {
                        return String.format(Locale.ENGLISH, "%.2f", capLeftPosition);
                    }
                })
                .addData("capRight", new Func<String>() {
                    @Override
                    public String value() {
                        return String.format(Locale.ENGLISH, "%.2f", capRightPosition);
                    }
                });

    }
}