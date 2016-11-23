package org.firstinspires.ftc.teamcode;

import java.util.Locale;

import static java.lang.Thread.sleep;
import static org.firstinspires.ftc.teamcode.measurements.mmPerInch;
import static org.firstinspires.ftc.teamcode.measurements.pi;
import static org.firstinspires.ftc.teamcode.robotconfig.dl;

/**
 * Created by mail2 on 11/15/2016.
 * Project: ftc_app_for_2016_robot
 */

class stateslist {

    public static robotconfig robot = new robotconfig();
    static int currentState;
    public int color = 0;
    public preciseMovement p = new preciseMovement();
    /***
     * state uses the light sensor to strafe towards the tape line
     */
    state scanForLine = new state("scanForLine") {
        public void firstTime() {
            robot.enableMotorBreak();
            robot.move(0, color * 0.3, 0);
        }

        public void everyTime() {

        }

        public boolean conditionsToCheck() {
            return robot.detectLine();
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
            robot.disableMotorBreak();
            robot.move(0.25, 0, 0);
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
                return robot.touchBeacon.isPressed();
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
                sleep(500);
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

            //Inner radius 49.25"
            //Inner arc length 49.25*pi/2
            //Outer radius 63.25"
            //Outer arc length 63.25*pi/2
            //Mid radius 56.25"
            //Mid arc length 56.25*pi/2

            if (color == 1)
                robot.setMyMotorTankTargets(p.mm2pulses(mmPerInch * 30 * pi / 2), p.mm2pulses(mmPerInch * 60 * pi / 2));
            else
                robot.setMyMotorTankTargets(p.mm2pulses(mmPerInch * 60 * pi / 2), p.mm2pulses(mmPerInch * 30 * pi / 2));

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

    state slideToTheRight = new state("slideToTheRight") {
        public void firstTime() {
            robot.setMyMotorTargets(0, color * p.mm2pulses(50 * mmPerInch), 0);
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
    private int startEncoderPos = 0;
    /***
     * state makes robot back away from beacon slightly to avoid running into anything during next state
     */
    state backAwayFromBeacon = new state("backAwayFromBeacon") {
        public void firstTime() {
            robot.enableMotorBreak();
            robot.move(-1, 0, 0);
            startEncoderPos = robot.getMotorEncoderAverage();
        }

        public void everyTime() {

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
                return robot.getMotorEncoderAverage() < p.mm2pulses(-3 * mmPerInch) + startEncoderPos;
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

    /***
     * state makes the robot attempt to knock over the cap ball
     */
    state retreatToCenter = new state("retreatToCenter") {

        public void firstTime() {
            robot.disableMotorBreak();
            startEncoderPos = robot.getMotorEncoderAverage();
            robot.move(-1, -color, 0);
        }

        public void everyTime() {

        }

        public boolean conditionsToCheck() {

            robotconfig.addlog(dl, "in retreatToCenter", "checking against robot.getMotorEncoderAverage() - startEncoderPos) * 2 < -p.mm2pulses(55 * mmPerInch ");

            if (robotconfig.debugMode) {
                if (this.isFirstTimeDebug) {
                    robotconfig.addlog(dl, "in retreatToCenter", "returning true");
                    return (true);
                } else {
                    this.isFirstTimeDebug = true;
                    robotconfig.addlog(dl, "in retreatToCenter", "returning false");
                    return (false);
                }
            } else {
                robotconfig.addlog(dl, "in retreatToCenter", "checking robot.getMotorEncoderAverage(): " + String.format(Locale.ENGLISH, "%d", robot.getMotorEncoderAverage()));
                return robot.getMotorEncoderAverage() > p.mm2pulses(3 * mmPerInch + 7 * mmPerInch * pi + 7 * mmPerInch) + startEncoderPos;
            }

        }

        public void onCompletion() {

        }
    };
    /***
     * state makes robot move backwards to try to park partially on the center vortex
     */
    state driveOnToWood = new state("driveOnToWood ") {
        public void firstTime() {
            robot.enableMotorBreak();
            robot.move(-1, 0, 0);
            startEncoderPos = robot.getMotorEncoderAverage();
        }

        public void everyTime() {

        }

        public boolean conditionsToCheck() {

            robotconfig.addlog(dl, "in driveOnToWood", "checking against robot.getMotorEncoderAverage() - startEncoderPos) * 2 < -p.mm2pulses(4 * mmPerInch ");

            if (robotconfig.debugMode) {
                if (this.isFirstTimeDebug) {
                    robotconfig.addlog(dl, "in driveOnToWood", "returning true");
                    return (true);
                } else {
                    this.isFirstTimeDebug = true;
                    robotconfig.addlog(dl, "in driveOnToWood", "returning false");
                    return (false);
                }
            } else {
                robotconfig.addlog(dl, "in driveOnToWood", "checking robot.getMotorEncoderAverage(): " + String.format(Locale.ENGLISH, "%d", robot.getMotorEncoderAverage()));
                return (robot.getMotorEncoderAverage() - startEncoderPos) * 2 < -p.mm2pulses(4 * mmPerInch);
            }

        }

        public void onCompletion() {
            robot.move(0, 0, 0);
        }
    };
}
