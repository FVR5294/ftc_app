package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Func;

import java.util.Locale;

import static org.firstinspires.ftc.teamcode.robotconfig.dl;
import static org.firstinspires.ftc.teamcode.superText.bars;
import static org.firstinspires.ftc.teamcode.superText.numbers;

/**
 * the main TeleOp program for our robot
 */

@TeleOp(name = "TeleOp_Revised", group = "2016")

public class TeleOp_Revised extends OpMode {

    final double buttonPusher_MIN_RANGE = 0.35;
    final double buttonPusher_MAX_RANGE = 0.65;

    final double Tilt_MAX_RANGE = 1.0;
    final double Tilt_MIN_RANGE = 1.0-140.0/255.0;

    final double capRight_MAX_RANGE = 0.95;
    final double capRight_MIN_RANGE = 0.05;

    final double capLeft_MAX_RANGE = 0.95;
    final double capLeft_MIN_RANGE = 0.05;
    final double buttonPusherDelta = 0.02;
    final double tiltDelta = 0.02;
    final double capLeftDelta = 0.02;
    final double capRightDelta = 0.02;

    final double pulses = 2240.0;
    final double pulsesReduced = pulses * 0.9;
    final double rampNumb = 3.5 / pulses;
    final boolean squishBallsTogether = false;

    final boolean illegibleText = false;
    final boolean barGraph = true;
    int completeSeconds = 0;
    robotconfig robot = new robotconfig();
    double endpulses = 0.0;
    double buttonPusherPosition = 0;
    double tiltPosition = 1.0-140.0/255.0;
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
    int scoreX = 0;
    int scoreB = 0;
    boolean endGame = false;
    boolean gp2x = false;
    boolean gp2b = false;
    boolean unleash = false;
    boolean color = false;
    int colorThreshold = 1;
    ElapsedTime eject = new ElapsedTime();
    ElapsedTime matchTimer = new ElapsedTime();
    int[] timerIndexes = {1, 2, 10, 3, 4};
    int[] scoreIndexes = {1, 2, 12, 3, 4};
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

    double intake4time = 0.1;
    ElapsedTime loadTimer = new ElapsedTime();
    ElapsedTime intake4timer = new ElapsedTime();
    boolean intake3 = false;
    //if ball is touching or beyond first limit switch
    boolean ballPresent = false;
    //if ball is beyond third switch
    boolean ballLoad = false;
    ElapsedTime constantRunTimer = new ElapsedTime();

    @Override
    public void init() {

//        robotconfig.debugMode = true;
        robot.init(this);

//        robot.disableMotorEncoders();
        // Send telemetry message to signify robot waiting;
//        robot.capLeft.getController().pwmDisable();
        telemetry.update();
        buttonPusherPosition = 0.5;
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
        loopTimer.reset();
        constantRunTimer.reset();
    }

    @Override
    public void loop() {

        loopTime = loopTimer.milliseconds();
        loopTimer.reset();
        totalLoopTime += loopTime;
        loopCount++;
        maxLoopTime = Math.max(maxLoopTime, loopTime);

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

        if (spinnerState)
            spinner = 1;
        else
            spinner = 0;

        if (robot.autoIntake) {

            if (squishBallsTogether) {

                if (!ballPresent)
                    if (robot.intake(1) || robot.intake(2) || robot.intake(3))
                        ballPresent = true;

            } else {

                if (!ballPresent && robot.intake(1)) {
                    ballPresent = true;
                    intake3 = !robot.intake(2);
                }

                if (intake3 && robot.intake(1) && robot.intake(2))
                    intake3 = false;

                if (ballPresent && robot.intake(2) && (robot.intake(3) || intake3)) {
                    ballPresent = false;
                    intake3 = true;
                }

            }

            //automatically stop bringing balls to the top
            if (unleash && robot.intake(3))
                unleash = false;

            if (robot.intake(3))
                loadTimer.reset();

            if (robot.intake(4))
                intake4timer.reset();

            if (robot.intake(1) || robot.intake(2) || robot.intake(3))
                constantRunTimer.reset();

            //check if ball entered puncher thing
            if (!ballLoad)
                if (intake4timer.seconds() < 0.1 || (vexes > 0.5 && loadTimer.seconds() > 0 && loadTimer.seconds() < 1))
                    ballLoad = true;

            if (ballLoad)
                if ((loadTimer.seconds() < 0.5 || loadTimer.seconds() > 1) && intake4timer.seconds() > 0.5)
                    ballLoad = false;
        }

        vexes = -gamepad2.left_stick_y / 2 + 0.5 + gamepad2.right_trigger / 2 - gamepad2.left_trigger / 2;

        if (robot.autoIntake && Math.abs(vexes - 0.5) < 0.1 && constantRunTimer.seconds() < 4) {

            if (squishBallsTogether) {

                if (!ballLoad || !robot.intake(3))
                    vexes = 1;

                if (spinnerState && robot.intake(3))
                    spinner = 0;

            } else {

                if (!ballLoad || (ballPresent && !robot.intake(3)))
                    vexes = 1;

                if (spinnerState && (ballPresent || (robot.intake(1) && robot.intake(2) && robot.intake(3))))
                    spinner = 0;

            }
        }

        if (robot.eject) {
            if (color) {
                if (robot.intake.blue() - robot.intake.red() > colorThreshold) {
                    spinner = -1;
                    eject.reset();
                }
            } else {
                if (robot.intake.red() - robot.intake.blue() > colorThreshold) {
                    spinner = -1;
                    eject.reset();
                }
            }

            if (eject.seconds() < 1) {
                spinner = -1;
                if (!ballPresent)
                    vexes = 0;
            }
        }

        robot.rvex.setPosition(vexes);
        robot.lvex.setPosition(vexes);

        if (gamepad1.left_trigger > 0.1 || gamepad1.right_trigger > 0.1)
            spinner = gamepad1.right_trigger - gamepad1.left_trigger;

        robot.spinner.setPower(spinner);

        if (!endGame && ballLoad && !robot.intake(4))
            robot.theHammerOfDawn.setPosition(1);
        else {
            if (vexes > 0.55)
                robot.theHammerOfDawn.setPosition(1);
            else if (vexes < 0.45)
                robot.theHammerOfDawn.setPosition(0);
            else
                robot.theHammerOfDawn.setPosition(vexes);
        }


        if (gamepad1.left_bumper || gamepad2.left_bumper) {
            if (!previousAState) {
                if (spinnerState) {
                    spinnerState = false;
                } else {
                    spinnerState = true;
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
            if (ballLoad && robot.garry.isPressed() && !previousGaryState) {
                ballLoad = false;
                if (squishBallsTogether)
                    ballPresent = false;
            }
        }

        if (gamepad1.y && !puncherState) {
//            if (!robot.garry.isPressed())
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

            if (gamepad2.x && !gp2x)
                if (gamepad2.right_bumper && scoreX > 0)
                    scoreX--;
                else
                    scoreX++;

            if (gamepad2.b && !gp2b)
                if (gamepad2.right_bumper && scoreB > 0)
                    scoreB--;
                else
                    scoreB++;

        }

        gp2x = gamepad2.x;
        gp2b = gamepad2.b;

        if (endGame && gamepad2.a) {
            tiltPosition -= tiltDelta;
            tiltPosition = Range.clip(tiltPosition, Tilt_MIN_RANGE, Tilt_MAX_RANGE);
            final double x1 = 1.0-140.0/255.0;
            final double x3 = 1.0;
            final double y1 = 0.0;
            final double y3 = 1.0-30.0/255.0;
            tilt2Position = (tiltPosition-x1)*(y3-y1)/(x3-x1) + y1;
            robot.tilt.setPosition(tiltPosition);
            robot.tilt2.setPosition(tilt2Position);
        } else if (gamepad2.y) {
            endGame = true;
            tiltPosition += tiltDelta;
            tiltPosition = Range.clip(tiltPosition, Tilt_MIN_RANGE, Tilt_MAX_RANGE);
            final double x1 = 1.0-140.0/255.0;
            final double x3 = 1.0;
            final double y1 = 0.0;
            final double y3 = 1.0-30.0/255.0;
            tilt2Position = (tiltPosition-x1)*(y3-y1)/(x3-x1) + y1;
            robot.tilt.setPosition(tiltPosition);
            robot.tilt2.setPosition(tilt2Position);
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

    void activateTelemetry() {
        telemetry.addAction(new Runnable() {
            @Override
            public void run() {
                if (matchTimer.seconds() < 120)
                    seconds = 120 - (int) matchTimer.seconds();
                else
                    seconds = (int) matchTimer.seconds();

                completeSeconds = seconds % 120;
                minutes = seconds;
                seconds %= 60;
                minutes -= seconds;
                minutes /= 60;

                if (illegibleText) {
                    timerIndexes[0] = minutes / 10;
                    timerIndexes[1] = minutes % 10;
                    timerIndexes[3] = seconds / 10;
                    timerIndexes[4] = seconds % 10;

                    scoreIndexes[0] = scoreX / 10;
                    scoreIndexes[1] = scoreX % 10;
                    scoreIndexes[3] = scoreB / 10;
                    scoreIndexes[4] = scoreB % 10;
                }

            }
        });

        final double barCount = 5;
        final double total = 30;

        if (barGraph) {
            telemetry.addLine()
                    .addData("t1", new Func<String>() {
                        @Override
                        public String value() {
                            int index = (int) Math.max(0, Math.min(barCount, barCount * ((completeSeconds - total * 0) / total)));
                            return bars[index];
                        }
                    });
            telemetry.addLine()
                    .addData("t2", new Func<String>() {
                        @Override
                        public String value() {
                            int index = (int) Math.max(0, Math.min(barCount, barCount * ((completeSeconds - total * 1) / total)));
                            return bars[index];
                        }
                    });
            telemetry.addLine()
                    .addData("t3", new Func<String>() {
                        @Override
                        public String value() {
                            int index = (int) Math.max(0, Math.min(barCount, barCount * ((completeSeconds - total * 2) / total)));
                            return bars[index];
                        }
                    });
            telemetry.addLine()
                    .addData("t4", new Func<String>() {
                        @Override
                        public String value() {
                            int index = (int) Math.max(0, Math.min(barCount, barCount * ((completeSeconds - total * 3) / total)));
                            return bars[index];
                        }
                    });
            telemetry.addLine();
        }

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

        telemetry.addLine();

        telemetry.addLine()
                .addData("score", new Func<String>() {
                    @Override
                    public String value() {
                        return String.format(Locale.ENGLISH, "%d - %d", scoreX, scoreB);
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
            if (robot.autoIntake)
                telemetry.addLine()
                        .addData("ballPresent", new Func<String>() {
                                    @Override
                                    public String value() {
                                        return String.format(Locale.ENGLISH, "%b", ballPresent);
                                    }
                                }
                        )
                        .addData("ballLoaded", new Func<String>() {
                            @Override
                            public String value() {
                                return String.format(Locale.ENGLISH, "%b", ballLoad);
                            }
                        });
            if (robot.autoIntake)
                telemetry.addLine()
                        .addData("intake1", new Func<String>() {
                                    @Override
                                    public String value() {
                                        return String.format(Locale.ENGLISH, "%b", robot.intake(1));
                                    }
                                }
                        )
                        .addData("intake2", new Func<String>() {
                                    @Override
                                    public String value() {
                                        return String.format(Locale.ENGLISH, "%b", robot.intake(2));
                                    }
                                }
                        )
                        .addData("intake3", new Func<String>() {
                                    @Override
                                    public String value() {
                                        return String.format(Locale.ENGLISH, "%b", robot.intake(3));
                                    }
                                }
                        )
                        .addData("intake4", new Func<String>() {
                            @Override
                            public String value() {
                                return String.format(Locale.ENGLISH, "%b", robot.intake(4));
                            }
                        });
        }
    }
}