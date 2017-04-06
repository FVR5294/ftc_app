package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Func;

import java.util.Locale;

import static org.firstinspires.ftc.teamcode.robotconfig.dl;
import static org.firstinspires.ftc.teamcode.superText.numbers;

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
    private final boolean illegibleText = false;
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
    private double vexes = 0.5;
    private double puncher = 0;
    private int minutes = 0;
    private int seconds = 0;
    private int scoreA = 0;
    private int scoreB = 0;
    private boolean endGame = false;
    private boolean gp2a = false;
    private boolean gp2b = false;
    private boolean unleash = false;
    private boolean color = false;
    private int colorThreshold = 1;
    private ElapsedTime eject = new ElapsedTime();
    private ElapsedTime matchTimer = new ElapsedTime();
    private int[] timerIndexes = {1, 2, 10, 3, 4};
    private int[] scoreIndexes = {1, 2, 12, 3, 4};
    private boolean puncherBroken = false;
    //if ball is somewhere between first two limit switches first switch inclusive second exclusive
    private boolean ballPresent = false;
    //if ball is beyond third switch
    private boolean ballLoad = false;
    private boolean previous3state = false;
    private boolean previous2state = false;
    private boolean previous1state = false;
    private boolean previousAState = false;
    private boolean previousGaryState = false;
    private boolean spinnerState = false;
    private boolean puncherState = false;
    private boolean speedToggleFlag = false;
    private boolean slowState = false;
    private ElapsedTime loopTimer = new ElapsedTime();

    private double loopTime = 0;

    @Override
    public void init() {

//        robotconfig.debugMode = true;
        robot.init(this);

//        robot.disableMotorEncoders();
        // Send telemetry message to signify robot waiting;
//        robot.capLeft.getController().pwmDisable();

        telemetry.update();
        buttonPusherPosition = 0.5;
        tiltPosition = 0.50;
        capLeftPosition = 0.05;
        capRightPosition = 0.05;
        loopTimer.reset();
        activateTelemetry();
    }

    @Override
    public void init_loop() {
        if (gamepad1.a)
            color = true;
        else if (gamepad1.b)
            color = false;
    }

    @Override
    public void start() {
        matchTimer.reset();
    }

    @Override
    public void loop() {

        loopTime = loopTimer.milliseconds();
        loopTimer.reset();


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

        if (spinnerState)
            spinner = 1;
        else
            spinner = 0;

        if (robot.autoIntake) {

            //remember if ball entered first stage
            if (!ballPresent && robot.intake(1))
                ballPresent = true;

            //automatically stop bringing balls to the top
            if (unleash && robot.intake(3))
                unleash = false;

            //check if ball left first stage in reverse
            if (ballPresent && previous1state && !robot.intake(1) && vexes < 0.5)
                ballPresent = false;

            //check if ball left first stage going forward
            if (ballPresent && previous2state && !robot.intake(2) && vexes > 0.5)
                ballPresent = false;

            //check if ball entered first stage in reverse
            if (!ballPresent && robot.intake(2))
                ballPresent = true;

            //check if ball entered puncher thing
            if (previous3state && !robot.intake(3) && vexes > 0.5)
                ballLoad = true;

            previous1state = robot.intake(1);
            previous2state = robot.intake(2);
            previous3state = robot.intake(3);
        }

        vexes = -gamepad2.left_stick_y * 0.5 + 0.5 + gamepad1.right_trigger / 2 + gamepad2.right_trigger / 2 - gamepad2.left_trigger / 2;

        if (robot.autoIntake && Math.abs(vexes - 0.5) < 0.1) {

            if (!ballLoad || ((ballPresent || unleash) && !robot.intake(3)))
                vexes = 1;

            if (spinnerState && (ballPresent || !ballLoad))
                spinner = 0;
        }

        if (robot.eject) {
            if (color) {
                if (robot.intake.blue() - robot.intake.red() > colorThreshold) {
                    spinner = -1;
                    if (robot.intake(1))
                        vexes = 0;
                    eject.reset();
                }
            } else {
                if (robot.intake.red() - robot.intake.blue() > colorThreshold) {
                    spinner = -1;
                    if (robot.intake(1))
                        vexes = 0;
                    eject.reset();
                }
            }
            if (eject.seconds() < 1) {
                spinner = -1;
                if (robot.autoIntake)
                    if (ballLoad)
                        vexes = 0;
            }
        }

        robot.rvex.setPosition(vexes);
        robot.lvex.setPosition(vexes);

        if (gamepad1.left_trigger > 0.1 || gamepad1.right_trigger > 0.1)
            spinner = gamepad1.right_trigger - gamepad1.left_trigger;

        robot.spinner.setPower(spinner);

        //automatically start when ball is above the L
        if (ballLoad)
            robot.theHammerOfDawn.setPosition(1);
        else
            robot.theHammerOfDawn.setPosition(vexes);

        //bring up next stored ball to top of the L
        if (gamepad2.a && !endGame)
            unleash = true;

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

        if (puncherState) {
            if ((!robot.garry.isPressed() && previousGaryState) || (puncherBroken && endpulses - robot.puncher.getCurrentPosition() < -100)) {
                robot.puncher.setPower(0);
                puncherState = false;
            } else {
                puncher = Math.min(1, Math.max(0.6, (endpulses - robot.puncher.getCurrentPosition()) * rampNumb));
                robot.puncher.setPower(puncher);
            }

            //tell auto intake script to go fetch a ball
            if (ballLoad && robot.garry.isPressed() && !previousGaryState)
                ballLoad = false;
        }

        if (gamepad1.y && !puncherState) {
            unleash = true;
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

        if (endGame) {

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

        } else {

            if (gamepad2.a && !gp2a)
                if (gamepad2.right_bumper && scoreA > 0)
                    scoreA--;
                else
                    scoreA++;

            if (gamepad2.b && !gp2b)
                if (gamepad2.right_bumper && scoreB > 0)
                    scoreB--;
                else
                    scoreB++;

        }

        gp2a = gamepad2.a;
        gp2b = gamepad2.b;

        if (endGame && gamepad2.a) {
            tiltPosition -= tiltDelta;
            tiltPosition = Range.clip(tiltPosition, Tilt_MIN_RANGE, Tilt_MAX_RANGE);
            robot.tilt.setPosition(tiltPosition);
        } else if (gamepad2.y) {
            endGame = true;
            tiltPosition += tiltDelta;
            tiltPosition = Range.clip(tiltPosition, Tilt_MIN_RANGE, Tilt_MAX_RANGE);
            robot.tilt.setPosition(tiltPosition);
        } else if (gamepad2.right_bumper) {
            endGame = false;
        }

        //if anything fails...
        if (gamepad2.back) {
            robot.capLeft.getController().pwmDisable();
            robot.tilt.getController().pwmDisable();
            robot.buttonPusher.getController().pwmDisable();
            robot.autoIntake = false;
            robot.eject = false;
            puncherBroken = true;
            ballLoad = false;
        }

    }

    @Override
    public void stop() {
        robot.theHammerOfDawn.setPosition(0.5);
        robot.rvex.setPosition(0.5);
        robot.lvex.setPosition(0.5);
    }

    private void activateTelemetry() {
        telemetry.addAction(new Runnable() {
            @Override
            public void run() {
                if (matchTimer.seconds() < 120)
                    seconds = 120 - (int) matchTimer.seconds();
                else
                    seconds = (int) matchTimer.seconds();

                minutes = seconds;
                seconds %= 60;
                minutes -= seconds;
                minutes /= 60;

                if (illegibleText) {
                    timerIndexes[0] = minutes / 10;
                    timerIndexes[1] = minutes % 10;
                    timerIndexes[3] = seconds / 10;
                    timerIndexes[4] = seconds % 10;

                    scoreIndexes[0] = scoreA / 10;
                    scoreIndexes[1] = scoreA % 10;
                    scoreIndexes[3] = scoreB / 10;
                    scoreIndexes[4] = scoreB % 10;
                }

            }
        });

        if (illegibleText) {
            telemetry.addLine()
                    .addData("t0", new Func<String>() {
                        @Override
                        public String value() {
                            return numbers[timerIndexes[1]][0] + numbers[timerIndexes[2]][0] + numbers[timerIndexes[3]][0] + numbers[timerIndexes[4]][0];
                        }
                    });
            telemetry.addLine()
                    .addData("t1", new Func<String>() {
                        @Override
                        public String value() {
                            return numbers[timerIndexes[1]][1] + numbers[timerIndexes[2]][1] + numbers[timerIndexes[3]][1] + numbers[timerIndexes[4]][1];
                        }
                    });
            telemetry.addLine()
                    .addData("t2", new Func<String>() {
                        @Override
                        public String value() {
                            return numbers[timerIndexes[1]][2] + numbers[timerIndexes[2]][2] + numbers[timerIndexes[3]][2] + numbers[timerIndexes[4]][2];
                        }
                    });
            telemetry.addLine()
                    .addData("t3", new Func<String>() {
                        @Override
                        public String value() {
                            return numbers[timerIndexes[1]][3] + numbers[timerIndexes[2]][3] + numbers[timerIndexes[3]][3] + numbers[timerIndexes[4]][3];
                        }
                    });
            telemetry.addLine()
                    .addData("t4", new Func<String>() {
                        @Override
                        public String value() {
                            return numbers[timerIndexes[1]][4] + numbers[timerIndexes[2]][4] + numbers[timerIndexes[3]][4] + numbers[timerIndexes[4]][4];
                        }
                    });
            telemetry.addLine()
                    .addData("t5", new Func<String>() {
                        @Override
                        public String value() {
                            return numbers[timerIndexes[1]][5] + numbers[timerIndexes[2]][5] + numbers[timerIndexes[3]][5] + numbers[timerIndexes[4]][5];
                        }
                    });
            telemetry.addLine()
                    .addData("t6", new Func<String>() {
                        @Override
                        public String value() {
                            return numbers[timerIndexes[1]][6] + numbers[timerIndexes[2]][6] + numbers[timerIndexes[3]][6] + numbers[timerIndexes[4]][6];
                        }
                    });
            telemetry.addLine()
                    .addData("t7", new Func<String>() {
                        @Override
                        public String value() {
                            return numbers[timerIndexes[1]][7] + numbers[timerIndexes[2]][7] + numbers[timerIndexes[3]][7] + numbers[timerIndexes[4]][7];
                        }
                    });

            telemetry.addLine()
                    .addData("s0", new Func<String>() {
                        @Override
                        public String value() {
                            return numbers[scoreIndexes[1]][0] + numbers[scoreIndexes[2]][0] + numbers[scoreIndexes[4]][0];
                        }
                    });
            telemetry.addLine()
                    .addData("s1", new Func<String>() {
                        @Override
                        public String value() {
                            return numbers[scoreIndexes[1]][1] + numbers[scoreIndexes[2]][1] + numbers[scoreIndexes[4]][1];
                        }
                    });
            telemetry.addLine()
                    .addData("s2", new Func<String>() {
                        @Override
                        public String value() {
                            return numbers[scoreIndexes[1]][2] + numbers[scoreIndexes[2]][2] + numbers[scoreIndexes[4]][2];
                        }
                    });
            telemetry.addLine()
                    .addData("s3", new Func<String>() {
                        @Override
                        public String value() {
                            return numbers[scoreIndexes[1]][3] + numbers[scoreIndexes[2]][3] + numbers[scoreIndexes[4]][3];
                        }
                    });
            telemetry.addLine()
                    .addData("s4", new Func<String>() {
                        @Override
                        public String value() {
                            return numbers[scoreIndexes[1]][4] + numbers[scoreIndexes[2]][4] + numbers[scoreIndexes[4]][4];
                        }
                    });
            telemetry.addLine()
                    .addData("s5", new Func<String>() {
                        @Override
                        public String value() {
                            return numbers[scoreIndexes[1]][5] + numbers[scoreIndexes[2]][5] + numbers[scoreIndexes[4]][5];
                        }
                    });
            telemetry.addLine()
                    .addData("s6", new Func<String>() {
                        @Override
                        public String value() {
                            return numbers[scoreIndexes[1]][6] + numbers[scoreIndexes[2]][6] + numbers[scoreIndexes[4]][6];
                        }
                    });
            telemetry.addLine()
                    .addData("s7", new Func<String>() {
                        @Override
                        public String value() {
                            return numbers[scoreIndexes[1]][7] + numbers[scoreIndexes[2]][7] + numbers[scoreIndexes[4]][7];
                        }
                    });
        }

        telemetry.addLine()
                .addData("match time", new Func<String>() {
                    @Override
                    public String value() {
                        return String.format(Locale.ENGLISH, "%d:%d", minutes, seconds);
                    }
                });
        telemetry.addLine()
                .addData("score", new Func<String>() {
                    @Override
                    public String value() {
                        return String.format(Locale.ENGLISH, "%d - %d", scoreA, scoreB);
                    }
                });
        telemetry.addLine()
                .addData("loop time", new Func<String>() {
                    @Override
                    public String value() {
                        return String.format(Locale.ENGLISH, "%.2f", loopTime);
                    }
                });

        telemetry.addLine()
                .addData("colorRed", new Func<String>() {
                    @Override
                    public String value() {
                        return String.format(Locale.ENGLISH, "%b", color);
                    }
                });

        if (!robotconfig.debugMode) {
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
                    .addData("capLeft", new Func<String>() {
                        @Override
                        public String value() {
                            return String.format(Locale.ENGLISH, "%.2f", capLeftPosition);
                        }
                    })
                    .addData("capRight", new Func<String>() {
                        @Override
                        public String value() {
                            return String.format(Locale.ENGLISH, "%.2f", capLeftPosition);
                        }
                    });
            if (robot.eject)
                telemetry.addLine()
                        .addData("red", new Func<String>() {
                                    @Override
                                    public String value() {
                                        return String.format(Locale.ENGLISH, "%d", robot.intake.red());
                                    }
                                }
                        )
                        .addData("blue", new Func<String>() {
                            @Override
                            public String value() {
                                return String.format(Locale.ENGLISH, "%d", robot.intake.blue());
                            }
                        });
        }
    }
}