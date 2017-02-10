package org.firstinspires.ftc.teamcode;

import static java.lang.Thread.sleep;
import static org.firstinspires.ftc.teamcode.measurements.mmPerInch;
import static org.firstinspires.ftc.teamcode.measurements.pi;
import static org.firstinspires.ftc.teamcode.robotconfig.dl;

/**
 * Created by mail2 on 11/15/2016.
 * Project: ftc_app_for_2016_robot
 */

/***
 * list of states for the state machine
 * each state can be referenced in other programs for use in various autonomous programs
 */
class stateslist {

    public static robotconfig robot = new robotconfig();//import the robot configuration
    static int currentState;//variable is used to control which state the state machine is currently running
    public int color = 0;//variable is used to change the behavior of the state depending on it's value of either 1 for red or -1 for blue
    public preciseMovement p = new preciseMovement();//import methods for precise movement
    /***
     * state uses the light sensor to strafe towards the tape line
     */
    state scanForLine = new state("scanForLine") {
        public void firstTime() {
            robot.move(0, color * 0.2, 0);
            while (!robot.detectLine()) {
                Thread.yield();
            }
        }

        public void everyTime() {

        }

        public boolean conditionsToCheck() {
            return true;
        }

        public void onCompletion() {
            robot.move(0, 0, 0);
        }
    };
    /***
     * state makes robot drive forward until touch sensor is touching beacon
     */
    state driveTowardsBeacon = new state("driveTowardsBeacon") {
        public void firstTime() {
            robot.enableEncodersToPosition();
            Thread.yield();
            robot.setMotorPower(1);
            Thread.yield();
            p.automaticSquareUp(robot);
            Thread.yield();
            robot.setMotorPower(0);
            Thread.yield();
            robot.enableMotorEncoders();
            Thread.yield();
            robot.move(0.2, 0, 0);
            while (!robot.touchBeacon.isPressed()) {
                Thread.yield();
            }
        }

        public void everyTime() {

        }

        public boolean conditionsToCheck() {

            //robotconfig.addlog(dl, "in driveTowardsBeacon", "checking against robot.touchBeacon.isPressed()");

            if (robotconfig.debugMode) {
                if (this.isFirstTimeDebug) {
                    robotconfig.addlog(dl, "in driveTowardsBeacon", "returning true");
                    return (true);
                } else {
                    this.isFirstTimeDebug = true;
                    robotconfig.addlog(dl, "in driveTowardsBeacon", "returning false");
                    return (false);
                }
            } else {
                return true;
            }

        }

        public void onCompletion() {
            robot.move(0, 0, 0);
        }
    };
    /***
     * state makes robot use color sensor and servo to try to press the button on the beacon
     */
    state pushBeaconButton = new state("pushBeaconButton") {
        public void firstTime() {

        }

        public void everyTime() {

        }

        public boolean conditionsToCheck() {
            return true;
        }

        public void onCompletion() {
            robot.pushButton(robot.detectColor() * color);
            try {
                sleep(1000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    };
    /***
     * state makes robot drive forward slightly
     */
    state clearWall = new state("clearWall") {

        public void firstTime() {
            //robot.setMyMotorTargets(p.mm2pulses(3 * mmPerInch), 0, 0);
        }

        public void everyTime() {
            //robotconfig.addlog(dl, "in clearWall", "error at, " + robot.getErrors());
            //robot.bettermove();
        }

        public boolean conditionsToCheck() {

            if (robotconfig.debugMode) {
                if (this.isFirstTimeDebug) {
                    robotconfig.addlog(dl, "in clearWall", "returning true");
                    return (true);
                } else {
                    this.isFirstTimeDebug = true;
                    robotconfig.addlog(dl, "in clearWall", "returning false");
                    return (false);
                }
            } else {
                return true;
                //return robot.bettermoving();
            }
        }

        public void onCompletion() {

        }
    };
    /***
     * state makes robot arc 90 degrees so it ends up pointed towards the beacon
     */
    state arcTowardsBeacon = new state("arcTowardsBeacon") {


        public void firstTime() {

            robot.enableMotorBreak();

            //Inner radius 49.25"
            //Inner arc length 49.25*pi/2
            //Outer radius 63.25"
            //Outer arc length 63.25*pi/2
            //Mid radius 56.25"
            //Mid arc length 56.25*pi/2

            if (color == 1)
                robot.setMyMotorTankTargets(p.mm2pulses(mmPerInch * 28 * pi / 2), p.mm2pulses(mmPerInch * 58 * pi / 2));
            else
                robot.setMyMotorTankTargets(p.mm2pulses(mmPerInch * 58 * pi / 2), p.mm2pulses(mmPerInch * 28 * pi / 2));

        }

        public void everyTime() {
            robot.bettermove();
            //robotconfig.addlog(dl, "in arcTowardsBeacon", "error at " + robot.getErrors());
        }

        public boolean conditionsToCheck() {
            //robotconfig.addlog(dl, "in arcTowardsBeacon", "checking against p.mm2pulses(3 * mmPerInch + 7 * mmPerInch * pi) + startEncoderPos: " + String.format(Locale.ENGLISH, "%d", p.mm2pulses(3 * mmPerInch + 7 * mmPerInch * pi) + startEncoderPos));

            if (robotconfig.debugMode) {
                if (this.isFirstTimeDebug) {
                    robotconfig.addlog(dl, "in arcTowardsBeacon", "returning true");
                    return (true);
                } else {
                    this.isFirstTimeDebug = true;
                    robotconfig.addlog(dl, "in arcTowardsBeacon", "returning false");
                    return (false);
                }
            } else {
                return robot.bettermoving();
            }
        }

        public void onCompletion() {
            robot.move(0, 0, 0);
            robot.enableEncodersToPosition();
            Thread.yield();
            robot.setMotorPower(1);
            Thread.yield();
            p.automaticSquareUp(robot);
            Thread.yield();
            robot.setMotorPower(0);
            Thread.yield();
            robot.enableMotorEncoders();
            Thread.yield();
        }
    };

    /***
     * state pivots robot about 45 degrees towards center vortex
     */
    state pivotbeacon = new state("pivotbeacon") {


        public void firstTime() {

            if (color == 1)
                robot.setMyMotorTankTargets(0, p.mm2pulses(mmPerInch * -6 * pi));
            else
                robot.setMyMotorTankTargets(p.mm2pulses(mmPerInch * -6 * pi), 0);

        }

        public void everyTime() {
            robot.bettermove();
            //robotconfig.addlog(dl, "in arcTowardsBeacon", "error at " + robot.getErrors());
        }

        public boolean conditionsToCheck() {
            //robotconfig.addlog(dl, "in arcTowardsBeacon", "checking against p.mm2pulses(3 * mmPerInch + 7 * mmPerInch * pi) + startEncoderPos: " + String.format(Locale.ENGLISH, "%d", p.mm2pulses(3 * mmPerInch + 7 * mmPerInch * pi) + startEncoderPos));

            if (robotconfig.debugMode) {
                if (this.isFirstTimeDebug) {
                    robotconfig.addlog(dl, "in pivotbeacon", "returning true");
                    return (true);
                } else {
                    this.isFirstTimeDebug = true;
                    robotconfig.addlog(dl, "in pivotbeacon", "returning false");
                    return (false);
                }
            } else {
                return robot.bettermoving();
            }
        }

        public void onCompletion() {
        }
    };

    state slideToTheRight = new state("slideToTheRight") {
        public void firstTime() {
            robot.setMyMotorTargets(0, color * p.mm2pulses(54 * mmPerInch), 0);
        }

        public void everyTime() {
            robot.bettermove();
            //robotconfig.addlog(dl, "in slideToTheRight", "error at " + robot.getErrors());
        }

        public boolean conditionsToCheck() {
            //robotconfig.addlog(dl, "in arcTowardsBeacon", "checking against p.mm2pulses(3 * mmPerInch + 7 * mmPerInch * pi) + startEncoderPos: " + String.format(Locale.ENGLISH, "%d", p.mm2pulses(3 * mmPerInch + 7 * mmPerInch * pi) + startEncoderPos));

            if (robotconfig.debugMode) {
                if (this.isFirstTimeDebug) {
                    robotconfig.addlog(dl, "in slideToTheRight", "returning true");
                    return (true);
                } else {
                    this.isFirstTimeDebug = true;
                    robotconfig.addlog(dl, "in slideToTheRight", "returning false");
                    return (false);
                }
            } else {
                return robot.bettermoving();
            }
        }

        public void onCompletion() {
            robot.move(0, 0, 0);
            robot.enableEncodersToPosition();
            Thread.yield();
            robot.setMotorPower(1);
            Thread.yield();
            p.automaticSquareUp(robot);
            Thread.yield();
            robot.setMotorPower(0);
            Thread.yield();
            robot.enableMotorEncoders();
            Thread.yield();
        }
    };

    state backup1 = new state("backup1") {
        public void firstTime() {
            robot.setMyMotorTargets(p.mm2pulses(-30 * mmPerInch), 0, 0);
        }

        public void everyTime() {
            robot.bettermove();
            //robotconfig.addlog(dl, "in slideToTheRight", "error at " + robot.getErrors());
        }

        public boolean conditionsToCheck() {
            //robotconfig.addlog(dl, "in arcTowardsBeacon", "checking against p.mm2pulses(3 * mmPerInch + 7 * mmPerInch * pi) + startEncoderPos: " + String.format(Locale.ENGLISH, "%d", p.mm2pulses(3 * mmPerInch + 7 * mmPerInch * pi) + startEncoderPos));

            if (robotconfig.debugMode) {
                if (this.isFirstTimeDebug) {
                    robotconfig.addlog(dl, "in backup1", "returning true");
                    return (true);
                } else {
                    this.isFirstTimeDebug = true;
                    robotconfig.addlog(dl, "in backup1", "returning false");
                    return (false);
                }
            } else {
                return robot.bettermoving();
            }
        }

        public void onCompletion() {
            robot.move(0, 0, 0);
            robot.enableEncodersToPosition();
            Thread.yield();
            robot.setMotorPower(1);
            Thread.yield();
            p.automaticSquareUp(robot);
            Thread.yield();
            robot.setMotorPower(0);
            Thread.yield();
            robot.enableMotorEncoders();
            Thread.yield();
        }
    };

    state backup2 = new state("backup2") {
        public void firstTime() {
            robot.setMyMotorTargets(p.mm2pulses(-24 * mmPerInch), 0, 0);
        }

        public void everyTime() {
            robot.bettermove();
            //robotconfig.addlog(dl, "in slideToTheRight", "error at " + robot.getErrors());
        }

        public boolean conditionsToCheck() {
            //robotconfig.addlog(dl, "in arcTowardsBeacon", "checking against p.mm2pulses(3 * mmPerInch + 7 * mmPerInch * pi) + startEncoderPos: " + String.format(Locale.ENGLISH, "%d", p.mm2pulses(3 * mmPerInch + 7 * mmPerInch * pi) + startEncoderPos));

            if (robotconfig.debugMode) {
                if (this.isFirstTimeDebug) {
                    robotconfig.addlog(dl, "in backup2", "returning true");
                    return (true);
                } else {
                    this.isFirstTimeDebug = true;
                    robotconfig.addlog(dl, "in backup2", "returning false");
                    return (false);
                }
            } else {
                return robot.bettermoving();
            }
        }

        public void onCompletion() {
            robot.move(0, 0, 0);
            robot.enableEncodersToPosition();
            Thread.yield();
            robot.setMotorPower(1);
            Thread.yield();
            p.automaticSquareUp(robot);
            Thread.yield();
            robot.setMotorPower(0);
            Thread.yield();
            robot.enableMotorEncoders();
            Thread.yield();
        }
    };

    /***
     * state makes robot drive closer to the wall so the sensor is in range of the tape
     */
    state getCloserToWall = new state("getCloserToWall") {
        public void firstTime() {
            robot.setMyMotorTargets((int) (3 * mmPerInch), 0, 0);
        }

        public void everyTime() {
            robot.bettermove();
        }

        public boolean conditionsToCheck() {

            //robotconfig.addlog(dl, "in getCloserToWall", "checking against p.mm2pulses(3 * mmPerInch + 7 * mmPerInch * pi + 7 * mmPerInch) + startEncoderPos: " + String.format(Locale.ENGLISH, "%d", p.mm2pulses(3 * mmPerInch + 7 * mmPerInch * pi + 7 * mmPerInch) + startEncoderPos));

            if (robotconfig.debugMode) {
                if (this.isFirstTimeDebug) {
                    robotconfig.addlog(dl, "in getCloserToWall", "returning true");
                    return (true);
                } else {
                    this.isFirstTimeDebug = true;
                    robotconfig.addlog(dl, "in getCloserToWall", "returning false");
                    return (false);
                }
            } else {
                return robot.bettermoving();
            }

        }

        public void onCompletion() {

        }
    };
    /***
     * rotates robot 45 degrees clockwise if red, counter if blue
     */
    state rotate45 = new state("rotate45") {
        public void firstTime() {
            robot.setMyMotorTargets(0, 0, p.mm2pulses(p.spin2mm(45 * color)));
        }

        public void everyTime() {
            robot.bettermove();
        }

        public boolean conditionsToCheck() {
            if (robotconfig.debugMode) {
                if (this.isFirstTimeDebug) {
                    robotconfig.addlog(dl, "in rotate45", "returning true");
                    return (true);
                } else {
                    this.isFirstTimeDebug = true;
                    robotconfig.addlog(dl, "in rotate45", "returning false");
                    return (false);
                }
            } else {
                return robot.bettermoving();
            }
        }

        public void onCompletion() {
            robot.enableEncodersToPosition();
            Thread.yield();
            robot.setMotorPower(1);
            Thread.yield();
            p.automaticSquareUp(robot);
            Thread.yield();
            robot.setMotorPower(0);
            Thread.yield();
            robot.enableMotorEncoders();
            Thread.yield();
        }
    };

    /***
     * rotates robot 60 degrees clockwise if red, counter if blue
     */
    state rotate60 = new state("rotate60") {
        public void firstTime() {
            robot.setMyMotorTargets(0, 0, p.mm2pulses(p.spin2mm(64 * color)));
        }

        public void everyTime() {
            robot.bettermove();
        }

        public boolean conditionsToCheck() {
            if (robotconfig.debugMode) {
                if (this.isFirstTimeDebug) {
                    robotconfig.addlog(dl, "in rotate60", "returning true");
                    return (true);
                } else {
                    this.isFirstTimeDebug = true;
                    robotconfig.addlog(dl, "in rotate60", "returning false");
                    return (false);
                }
            } else {
                return robot.bettermoving();
            }
        }

        public void onCompletion() {
            robot.move(0, 0, 0);
        }
    };

    /***
     * rotates robot 60 degrees clockwise if red, counter if blue
     */
    state rotateN = new state("rotateN") {
        public void firstTime() {
            robot.setMyMotorTargets(0, 0, p.mm2pulses(p.spin2mm(40 * color)));
        }

        public void everyTime() {
            robot.bettermove();
        }

        public boolean conditionsToCheck() {
            if (robotconfig.debugMode) {
                if (this.isFirstTimeDebug) {
                    robotconfig.addlog(dl, "in rotateN", "returning true");
                    return (true);
                } else {
                    this.isFirstTimeDebug = true;
                    robotconfig.addlog(dl, "in rotateN", "returning false");
                    return (false);
                }
            } else {
                return robot.bettermoving();
            }
        }

        public void onCompletion() {
            robot.move(0, 0, 0);
        }
    };

    /***
     * rotates robot 90 degrees clockwise if red, counter if blue
     */
    state rotate90 = new state("rotate90") {
        public void firstTime() {
            robot.setMyMotorTargets(0, 0, p.mm2pulses(p.spin2mm(90 * color)));
        }

        public void everyTime() {
            robot.bettermove();
        }

        public boolean conditionsToCheck() {
            if (robotconfig.debugMode) {
                if (this.isFirstTimeDebug) {
                    robotconfig.addlog(dl, "in rotate90", "returning true");
                    return (true);
                } else {
                    this.isFirstTimeDebug = true;
                    robotconfig.addlog(dl, "in rotate90", "returning false");
                    return (false);
                }
            } else {
                return robot.bettermoving();
            }
        }

        public void onCompletion() {
            robot.enableEncodersToPosition();
            Thread.yield();
            robot.setMotorPower(1);
            Thread.yield();
            p.automaticSquareUp(robot);
            Thread.yield();
            robot.setMotorPower(0);
            Thread.yield();
            robot.enableMotorEncoders();
            Thread.yield();
        }
    };

    /***
     * backs up closer to te center vortex
     */
    state backuptovortex = new state("backuptovortex") {
        public void firstTime() {
            robot.setMyMotorTargets(p.mm2pulses(-46 * mmPerInch), 0, 0);
        }

        public void everyTime() {
            robot.bettermove(1);
        }

        public boolean conditionsToCheck() {
            if (robotconfig.debugMode) {
                if (this.isFirstTimeDebug) {
                    robotconfig.addlog(dl, "in backuptovortex", "returning true");
                    return (true);
                } else {
                    this.isFirstTimeDebug = true;
                    robotconfig.addlog(dl, "in backuptovortex", "returning false");
                    return (false);
                }
            } else {
                return robot.bettermoving();
            }
        }

        public void onCompletion() {
            robot.move(0, 0, 0);
        }
    };

    /***
     * strafes to the center vortex
     */
    state strafetovortex = new state("strafetovortex") {
        public void firstTime() {
            robot.setMyMotorTargets(p.mm2pulses(-48 * mmPerInch), p.mm2pulses(-48 * mmPerInch * color), 0);
        }

        public void everyTime() {
            robot.bettermove();
        }

        public boolean conditionsToCheck() {
            if (robotconfig.debugMode) {
                if (this.isFirstTimeDebug) {
                    robotconfig.addlog(dl, "in strafetovortex", "returning true");
                    return (true);
                } else {
                    this.isFirstTimeDebug = true;
                    robotconfig.addlog(dl, "in strafetovortex", "returning false");
                    return (false);
                }
            } else {
                return robot.bettermoving();
            }
        }

        public void onCompletion() {
            robot.move(0, 0, 0);
        }
    };

    /***
     * a state that does a 360 that probably will never be run
     */
    state noscope = new state("noscope") {
        public void firstTime() {
            robot.setMyMotorTargets(0, 0, p.mm2pulses(p.spin2mm(360 * color)));
        }

        public void everyTime() {
            robot.bettermove();
        }

        public boolean conditionsToCheck() {
            if (robotconfig.debugMode) {
                if (this.isFirstTimeDebug) {
                    robotconfig.addlog(dl, "in noscope", "returning true");
                    return (true);
                } else {
                    this.isFirstTimeDebug = true;
                    robotconfig.addlog(dl, "in noscope", "returning false");
                    return (false);
                }
            } else {
                return robot.bettermoving();
            }
        }

        public void onCompletion() {
            robot.enableEncodersToPosition();
            Thread.yield();
            robot.setMotorPower(1);
            Thread.yield();
            p.automaticSquareUp(robot);
            Thread.yield();
            robot.setMotorPower(0);
            Thread.yield();
            robot.enableMotorEncoders();
            Thread.yield();
        }
    };

    /***
     * spins puncher 360 degrees and runs vex motors
     */
    state shootball = new state("shootball") {

        public void firstTime() {
            robot.puncher.setPower(1);
            robot.lvex.setPosition(1);
            robot.rvex.setPosition(1);
            while (!robot.garry.isPressed()) {
                Thread.yield();
            }
            while (robot.garry.isPressed()) {
                Thread.yield();
            }
        }

        public void everyTime() {

        }

        public boolean conditionsToCheck() {
            if (robotconfig.debugMode) {
                if (this.isFirstTimeDebug) {
                    robotconfig.addlog(dl, "in shootball", "returning true");
                    return (true);
                } else {
                    this.isFirstTimeDebug = true;
                    robotconfig.addlog(dl, "in shootball", "returning false");
                    return (false);
                }
            } else {
                return true;
            }
        }

        public void onCompletion() {
            robot.puncher.setPower(0);
        }
    };

    /***
     * spins puncher 360 degrees and stops run vex motors
     */
    state shootball2 = new state("shootball2") {

        boolean previousGarry = false;

        public void firstTime() {
            robot.puncher.setPower(1);
            while (!robot.garry.isPressed()) {
                Thread.yield();
            }
            while (robot.garry.isPressed()) {
                Thread.yield();
            }
        }

        public void everyTime() {

        }

        public boolean conditionsToCheck() {
            if (robotconfig.debugMode) {
                if (this.isFirstTimeDebug) {
                    robotconfig.addlog(dl, "in shootball", "returning true");
                    return (true);
                } else {
                    this.isFirstTimeDebug = true;
                    robotconfig.addlog(dl, "in shootball", "returning false");
                    return (false);
                }
            } else {
                return true;
            }
        }

        public void onCompletion() {
            robot.puncher.setPower(0);
            robot.lvex.setPosition(0.5);
            robot.rvex.setPosition(0.5);
        }
    };

    /***
     * state corrects for error in robot strafing
     */
    state correctStrafe = new state("correctStrafe") {
        public void firstTime() {
            robot.setMyMotorTargets(p.mm2pulses(10 * mmPerInch), 0, 0);
        }

        public void everyTime() {
            robot.bettermove();
        }

        public boolean conditionsToCheck() {

            //robotconfig.addlog(dl, "in backAwayFromBeacon", "checking against p.mm2pulses(3 * mmPerInch + 7 * mmPerInch * pi + 7 * mmPerInch) + startEncoderPos: " + String.format(Locale.ENGLISH, "%d", p.mm2pulses(3 * mmPerInch + 7 * mmPerInch * pi + 7 * mmPerInch) + startEncoderPos));

            if (robotconfig.debugMode) {
                if (this.isFirstTimeDebug) {
                    robotconfig.addlog(dl, "in correctStrafe", "returning true");
                    return (true);
                } else {
                    this.isFirstTimeDebug = true;
                    robotconfig.addlog(dl, "in correctStrafe", "returning false");
                    return (false);
                }
            } else {
                //robotconfig.addlog(dl, "in backAwayFromBeacon", "checking robot.getMotorEncoderAverage(): " + String.format(Locale.ENGLISH, "%d", robot.getMotorEncoderAverage()));
                //return robot.getMotorEncoderAverage() < p.mm2pulses(-3 * mmPerInch) + startEncoderPos;
                return robot.bettermoving();
            }

        }

        public void onCompletion() {
            robot.move(0, 0, 0);
        }
    };
    /***
     * state makes robot back away from beacon slightly to avoid running into anything during next state and to aim
     */
    state backAwayFromBeacon = new state("backAwayFromBeacon") {
        public void firstTime() {
            robot.setMyMotorTargets(p.mm2pulses(-16 * mmPerInch), 0, 0);
        }

        public void everyTime() {
            robot.bettermove();
        }

        public boolean conditionsToCheck() {

            //robotconfig.addlog(dl, "in backAwayFromBeacon", "checking against p.mm2pulses(3 * mmPerInch + 7 * mmPerInch * pi + 7 * mmPerInch) + startEncoderPos: " + String.format(Locale.ENGLISH, "%d", p.mm2pulses(3 * mmPerInch + 7 * mmPerInch * pi + 7 * mmPerInch) + startEncoderPos));

            if (robotconfig.debugMode) {
                if (this.isFirstTimeDebug) {
                    robotconfig.addlog(dl, "in backAwayFromBeacon", "returning true");
                    return (true);
                } else {
                    this.isFirstTimeDebug = true;
                    robotconfig.addlog(dl, "in backAwayFromBeacon", "returning false");
                    return (false);
                }
            } else {
                //robotconfig.addlog(dl, "in backAwayFromBeacon", "checking robot.getMotorEncoderAverage(): " + String.format(Locale.ENGLISH, "%d", robot.getMotorEncoderAverage()));
                return robot.bettermoving();
            }

        }

        public void onCompletion() {
            robot.move(0, 0, 0);
            robot.pushButton(0);
            try {
                sleep(100);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            robot.enableEncodersToPosition();
            Thread.yield();
            robot.setMotorPower(1);
            Thread.yield();
            p.automaticSquareUp(robot);
            Thread.yield();
            robot.setMotorPower(0);
            Thread.yield();
            robot.enableMotorEncoders();
            Thread.yield();
        }
    };
}
