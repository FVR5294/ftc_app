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
        }

        public void everyTime() {
            robot.ultramove(0, color * 0.2, 30);
        }

        public boolean conditionsToCheck() {
            return robot.detectLine();
        }

        public void onCompletion() {
            robot.move(0, 0, 0);
        }
    };
    state scanForLineInverted = new state("scanForLineInverted") {
        public void firstTime() {
            robot.move(0, color * -0.2, 0);
        }

        public void everyTime() {
            robot.ultramove(0, color * -0.2, 30);
        }

        public boolean conditionsToCheck() {
            return robot.detectLine();
        }

        public void onCompletion() {
            robot.move(0, 0, 0);
        }
    };
    state colorRed = new state("colorRed") {
        public void firstTime() {
            color = 1;
        }
    };
    state colorBlue = new state("colorBlue") {
        public void firstTime() {
            color = -1;
        }
    };
    state sleep0 = new state("sleep0") {
        public void firstTime() {
            try {
                sleep(0);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    };
    state sleep500 = new state("sleep500") {
        public void firstTime() {
            try {
                sleep(500);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    };
    state sleep1000 = new state("sleep1000") {
        public void firstTime() {
            try {
                sleep(1000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    };
    state sleep2000 = new state("sleep2000") {
        public void firstTime() {
            try {
                sleep(2000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    };
    state sleep4000 = new state("sleep4000") {
        public void firstTime() {
            try {
                sleep(4000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    };
    state sleep10000 = new state("sleep10000") {
        public void firstTime() {
            try {
                sleep(10000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    };
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
        }

        public void everyTime() {
            robot.ultramove(0.2, 0);
        }

        public boolean conditionsToCheck() {
            return robot.touchBeacon.isPressed();
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

    state arc2 = new state("arc2") {


        public void firstTime() {

            robot.enableMotorBreak();

            //Inner radius 49.25"
            //Inner arc length 49.25*pi/2
            //Outer radius 63.25"
            //Outer arc length 63.25*pi/2
            //Mid radius 56.25"
            //Mid arc length 56.25*pi/2

            if (color == 1)
                robot.setMyMotorTankTargets(p.mm2pulses(mmPerInch * 50 * pi / 2), p.mm2pulses(mmPerInch * 3 * pi / 2));
            else
                robot.setMyMotorTankTargets(p.mm2pulses(mmPerInch * 3 * pi / 2), p.mm2pulses(mmPerInch * 50 * pi / 2));

        }

        public void everyTime() {
            robot.bettermove();
            //robotconfig.addlog(dl, "in arc2", "error at " + robot.getErrors());
        }

        public boolean conditionsToCheck() {
            //robotconfig.addlog(dl, "in arc2", "checking against p.mm2pulses(3 * mmPerInch + 7 * mmPerInch * pi) + startEncoderPos: " + String.format(Locale.ENGLISH, "%d", p.mm2pulses(3 * mmPerInch + 7 * mmPerInch * pi) + startEncoderPos));

            if (robotconfig.debugMode) {
                if (this.isFirstTimeDebug) {
                    robotconfig.addlog(dl, "in arc2", "returning true");
                    return (true);
                } else {
                    this.isFirstTimeDebug = true;
                    robotconfig.addlog(dl, "in arc2", "returning false");
                    return (false);
                }
            } else {
                return robot.bettermoving();
            }
        }

        public void onCompletion() {
            robot.move(0, 0, 0);
//            robot.enableEncodersToPosition();
//            Thread.yield();
//            robot.setMotorPower(1);
//            Thread.yield();
//            p.automaticSquareUp(robot);
//            Thread.yield();
//            robot.setMotorPower(0);
//            Thread.yield();
//            robot.enableMotorEncoders();
//            Thread.yield();
        }
    };

    state arcTowardsBeacon28 = new state("arcTowardsBeacon28") {


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
            //robotconfig.addlog(dl, "in arcTowardsBeacon28", "error at " + robot.getErrors());
        }

        public boolean conditionsToCheck() {
            //robotconfig.addlog(dl, "in arcTowardsBeacon28", "checking against p.mm2pulses(3 * mmPerInch + 7 * mmPerInch * pi) + startEncoderPos: " + String.format(Locale.ENGLISH, "%d", p.mm2pulses(3 * mmPerInch + 7 * mmPerInch * pi) + startEncoderPos));

            if (robotconfig.debugMode) {
                if (this.isFirstTimeDebug) {
                    robotconfig.addlog(dl, "in arcTowardsBeacon28", "returning true");
                    return (true);
                } else {
                    this.isFirstTimeDebug = true;
                    robotconfig.addlog(dl, "in arcTowardsBeacon28", "returning false");
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

    state arcTowardsBeacon24 = new state("arcTowardsBeacon24") {


        public void firstTime() {

            robot.enableMotorBreak();

            //Inner radius 49.25"
            //Inner arc length 49.25*pi/2
            //Outer radius 63.25"
            //Outer arc length 63.25*pi/2
            //Mid radius 56.25"
            //Mid arc length 56.25*pi/2

            if (color == 1)
                robot.setMyMotorTankTargets(p.mm2pulses(mmPerInch * 24 * pi / 2), p.mm2pulses(mmPerInch * 54 * pi / 2));
            else
                robot.setMyMotorTankTargets(p.mm2pulses(mmPerInch * 54 * pi / 2), p.mm2pulses(mmPerInch * 24 * pi / 2));

        }

        public void everyTime() {
            robot.bettermove();
            //robotconfig.addlog(dl, "in arcTowardsBeacon24", "error at " + robot.getErrors());
        }

        public boolean conditionsToCheck() {
            //robotconfig.addlog(dl, "in arcTowardsBeacon24", "checking against p.mm2pulses(3 * mmPerInch + 7 * mmPerInch * pi) + startEncoderPos: " + String.format(Locale.ENGLISH, "%d", p.mm2pulses(3 * mmPerInch + 7 * mmPerInch * pi) + startEncoderPos));

            if (robotconfig.debugMode) {
                if (this.isFirstTimeDebug) {
                    robotconfig.addlog(dl, "in arcTowardsBeacon24", "returning true");
                    return (true);
                } else {
                    this.isFirstTimeDebug = true;
                    robotconfig.addlog(dl, "in arcTowardsBeacon24", "returning false");
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

    state arcTowardsBeacon20 = new state("arcTowardsBeacon20") {


        public void firstTime() {

            robot.enableMotorBreak();

            //Inner radius 49.25"
            //Inner arc length 49.25*pi/2
            //Outer radius 63.25"
            //Outer arc length 63.25*pi/2
            //Mid radius 56.25"
            //Mid arc length 56.25*pi/2

            if (color == 1)
                robot.setMyMotorTankTargets(p.mm2pulses(mmPerInch * 20 * pi / 2), p.mm2pulses(mmPerInch * 50 * pi / 2));
            else
                robot.setMyMotorTankTargets(p.mm2pulses(mmPerInch * 50 * pi / 2), p.mm2pulses(mmPerInch * 20 * pi / 2));

        }

        public void everyTime() {
            robot.bettermove();
            //robotconfig.addlog(dl, "in arcTowardsBeacon20", "error at " + robot.getErrors());
        }

        public boolean conditionsToCheck() {
            //robotconfig.addlog(dl, "in arcTowardsBeacon20", "checking against p.mm2pulses(3 * mmPerInch + 7 * mmPerInch * pi) + startEncoderPos: " + String.format(Locale.ENGLISH, "%d", p.mm2pulses(3 * mmPerInch + 7 * mmPerInch * pi) + startEncoderPos));

            if (robotconfig.debugMode) {
                if (this.isFirstTimeDebug) {
                    robotconfig.addlog(dl, "in arcTowardsBeacon20", "returning true");
                    return (true);
                } else {
                    this.isFirstTimeDebug = true;
                    robotconfig.addlog(dl, "in arcTowardsBeacon20", "returning false");
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

    state arcTowardsBeacon32 = new state("arcTowardsBeacon32") {


        public void firstTime() {

            robot.enableMotorBreak();

            //Inner radius 49.25"
            //Inner arc length 49.25*pi/2
            //Outer radius 63.25"
            //Outer arc length 63.25*pi/2
            //Mid radius 56.25"
            //Mid arc length 56.25*pi/2

            if (color == 1)
                robot.setMyMotorTankTargets(p.mm2pulses(mmPerInch * 32 * pi / 2), p.mm2pulses(mmPerInch * 62 * pi / 2));
            else
                robot.setMyMotorTankTargets(p.mm2pulses(mmPerInch * 62 * pi / 2), p.mm2pulses(mmPerInch * 32 * pi / 2));

        }

        public void everyTime() {
            robot.bettermove();
            //robotconfig.addlog(dl, "in arcTowardsBeacon32", "error at " + robot.getErrors());
        }

        public boolean conditionsToCheck() {
            //robotconfig.addlog(dl, "in arcTowardsBeacon32", "checking against p.mm2pulses(3 * mmPerInch + 7 * mmPerInch * pi) + startEncoderPos: " + String.format(Locale.ENGLISH, "%d", p.mm2pulses(3 * mmPerInch + 7 * mmPerInch * pi) + startEncoderPos));

            if (robotconfig.debugMode) {
                if (this.isFirstTimeDebug) {
                    robotconfig.addlog(dl, "in arcTowardsBeacon32", "returning true");
                    return (true);
                } else {
                    this.isFirstTimeDebug = true;
                    robotconfig.addlog(dl, "in arcTowardsBeacon32", "returning false");
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

    state pivotbeaconmore = new state("pivotbeaconmore") {


        public void firstTime() {

            if (color == 1)
                robot.setMyMotorTankTargets(0, p.mm2pulses(mmPerInch * -12 * pi));
            else
                robot.setMyMotorTankTargets(p.mm2pulses(mmPerInch * -12 * pi), 0);

        }

        public void everyTime() {
            robot.bettermove();
            //robotconfig.addlog(dl, "in arcTowardsBeacon", "error at " + robot.getErrors());
        }

        public boolean conditionsToCheck() {
            //robotconfig.addlog(dl, "in arcTowardsBeacon", "checking against p.mm2pulses(3 * mmPerInch + 7 * mmPerInch * pi) + startEncoderPos: " + String.format(Locale.ENGLISH, "%d", p.mm2pulses(3 * mmPerInch + 7 * mmPerInch * pi) + startEncoderPos));

            if (robotconfig.debugMode) {
                if (this.isFirstTimeDebug) {
                    robotconfig.addlog(dl, "in pivotbeaconmore", "returning true");
                    return (true);
                } else {
                    this.isFirstTimeDebug = true;
                    robotconfig.addlog(dl, "in pivotbeaconmore", "returning false");
                    return (false);
                }
            } else {
                return robot.bettermoving();
            }
        }

        public void onCompletion() {
        }
    };

    state pivotbeaconless = new state("pivotbeaconless") {


        public void firstTime() {

            if (color == 1)
                robot.setMyMotorTankTargets(0, p.mm2pulses(mmPerInch * -4 * pi));
            else
                robot.setMyMotorTankTargets(p.mm2pulses(mmPerInch * -4 * pi), 0);

        }

        public void everyTime() {
            robot.bettermove();
            //robotconfig.addlog(dl, "in arcTowardsBeacon", "error at " + robot.getErrors());
        }

        public boolean conditionsToCheck() {
            //robotconfig.addlog(dl, "in arcTowardsBeacon", "checking against p.mm2pulses(3 * mmPerInch + 7 * mmPerInch * pi) + startEncoderPos: " + String.format(Locale.ENGLISH, "%d", p.mm2pulses(3 * mmPerInch + 7 * mmPerInch * pi) + startEncoderPos));

            if (robotconfig.debugMode) {
                if (this.isFirstTimeDebug) {
                    robotconfig.addlog(dl, "in pivotbeaconless", "returning true");
                    return (true);
                } else {
                    this.isFirstTimeDebug = true;
                    robotconfig.addlog(dl, "in pivotbeaconless", "returning false");
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
            robot.ultramove(30);
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
                return robot.ultramoving(30);
            }
        }

        public void onCompletion() {
            robot.move(0, 0, 0);
//            robot.enableEncodersToPosition();
//            Thread.yield();
//            robot.setMotorPower(1);
//            Thread.yield();
//            p.automaticSquareUp(robot);
//            Thread.yield();
//            robot.setMotorPower(0);
//            Thread.yield();
//            robot.enableMotorEncoders();
//            Thread.yield();
        }
    };

    state slideToTheRight2 = new state("slideToTheRight2") {
        public void firstTime() {
            robot.setMyMotorTargets(0, color * p.mm2pulses(54 * mmPerInch), 0);
        }

        public void everyTime() {
            robot.ultramoving(30);
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
                return robot.drunkmoving(30);
            }
        }

        public void onCompletion() {
            robot.move(0, 0, 0);
//            robot.enableEncodersToPosition();
//            Thread.yield();
//            robot.setMotorPower(1);
//            Thread.yield();
//            p.automaticSquareUp(robot);
//            Thread.yield();
//            robot.setMotorPower(0);
//            Thread.yield();
//            robot.enableMotorEncoders();
//            Thread.yield();
        }
    };

    state slideToTheRight54 = new state("slideToTheRight54") {
        public void firstTime() {
            robot.setMyMotorTargets(0, color * p.mm2pulses(54 * mmPerInch), 0);
        }

        public void everyTime() {
            robot.bettermove();
        }

        public boolean conditionsToCheck() {
            //robotconfig.addlog(dl, "in arcTowardsBeacon", "checking against p.mm2pulses(3 * mmPerInch + 7 * mmPerInch * pi) + startEncoderPos: " + String.format(Locale.ENGLISH, "%d", p.mm2pulses(3 * mmPerInch + 7 * mmPerInch * pi) + startEncoderPos));

            if (robotconfig.debugMode) {
                if (this.isFirstTimeDebug) {
                    robotconfig.addlog(dl, "in slideToTheRight54", "returning true");
                    return (true);
                } else {
                    this.isFirstTimeDebug = true;
                    robotconfig.addlog(dl, "in slideToTheRight54", "returning false");
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

    state slideToTheRight50 = new state("slideToTheRight50") {
        public void firstTime() {
            robot.setMyMotorTargets(0, color * p.mm2pulses(50 * mmPerInch), 0);
        }

        public void everyTime() {
            robot.bettermove();
        }

        public boolean conditionsToCheck() {
            //robotconfig.addlog(dl, "in arcTowardsBeacon", "checking against p.mm2pulses(3 * mmPerInch + 7 * mmPerInch * pi) + startEncoderPos: " + String.format(Locale.ENGLISH, "%d", p.mm2pulses(3 * mmPerInch + 7 * mmPerInch * pi) + startEncoderPos));

            if (robotconfig.debugMode) {
                if (this.isFirstTimeDebug) {
                    robotconfig.addlog(dl, "in slideToTheRight50", "returning true");
                    return (true);
                } else {
                    this.isFirstTimeDebug = true;
                    robotconfig.addlog(dl, "in slideToTheRight50", "returning false");
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

    state slideToTheRight58 = new state("slideToTheRight58") {
        public void firstTime() {
            robot.setMyMotorTargets(0, color * p.mm2pulses(58 * mmPerInch), 0);
        }

        public void everyTime() {
            robot.bettermove();
        }

        public boolean conditionsToCheck() {
            //robotconfig.addlog(dl, "in arcTowardsBeacon", "checking against p.mm2pulses(3 * mmPerInch + 7 * mmPerInch * pi) + startEncoderPos: " + String.format(Locale.ENGLISH, "%d", p.mm2pulses(3 * mmPerInch + 7 * mmPerInch * pi) + startEncoderPos));

            if (robotconfig.debugMode) {
                if (this.isFirstTimeDebug) {
                    robotconfig.addlog(dl, "in slideToTheRight58", "returning true");
                    return (true);
                } else {
                    this.isFirstTimeDebug = true;
                    robotconfig.addlog(dl, "in slideToTheRight58", "returning false");
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

    state slideToTheLeft = new state("slideToTheLeft") {
        public void firstTime() {
            robot.setMyMotorTargets(0, color * p.mm2pulses(-54 * mmPerInch), 0);
        }

        public void everyTime() {
            robot.ultramove(30);
        }

        public boolean conditionsToCheck() {
            //robotconfig.addlog(dl, "in arcTowardsBeacon", "checking against p.mm2pulses(3 * mmPerInch + 7 * mmPerInch * pi) + startEncoderPos: " + String.format(Locale.ENGLISH, "%d", p.mm2pulses(3 * mmPerInch + 7 * mmPerInch * pi) + startEncoderPos));

            if (robotconfig.debugMode) {
                if (this.isFirstTimeDebug) {
                    robotconfig.addlog(dl, "in slideToTheLeft", "returning true");
                    return (true);
                } else {
                    this.isFirstTimeDebug = true;
                    robotconfig.addlog(dl, "in slideToTheLeft", "returning false");
                    return (false);
                }
            } else {
                return robot.ultramoving(30);
            }
        }

        public void onCompletion() {
            robot.move(0, 0, 0);
//            robot.enableEncodersToPosition();
//            Thread.yield();
//            robot.setMotorPower(1);
//            Thread.yield();
//            p.automaticSquareUp(robot);
//            Thread.yield();
//            robot.setMotorPower(0);
//            Thread.yield();
//            robot.enableMotorEncoders();
//            Thread.yield();
        }
    };

    state backup30 = new state("backup30") {
        public void firstTime() {
            robot.setMyMotorTargets(p.mm2pulses(-30 * mmPerInch), 0, 0);
        }

        public void everyTime() {
            robot.gyromove();
            //robotconfig.addlog(dl, "in slideToTheRight", "error at " + robot.getErrors());
        }

        public boolean conditionsToCheck() {
            //robotconfig.addlog(dl, "in arcTowardsBeacon", "checking against p.mm2pulses(3 * mmPerInch + 7 * mmPerInch * pi) + startEncoderPos: " + String.format(Locale.ENGLISH, "%d", p.mm2pulses(3 * mmPerInch + 7 * mmPerInch * pi) + startEncoderPos));

            if (robotconfig.debugMode) {
                if (this.isFirstTimeDebug) {
                    robotconfig.addlog(dl, "in backup30", "returning true");
                    return (true);
                } else {
                    this.isFirstTimeDebug = true;
                    robotconfig.addlog(dl, "in backup30", "returning false");
                    return (false);
                }
            } else {
                return robot.gyromoving();
            }
        }

        public void onCompletion() {
            robot.move(0, 0, 0);
        }
    };

    state backup42 = new state("backup42") {
        public void firstTime() {
            robot.setMyMotorTargets(p.mm2pulses(-42 * mmPerInch), 0, 0);
        }

        public void everyTime() {
            robot.gyromove();
            //robotconfig.addlog(dl, "in slideToTheRight", "error at " + robot.getErrors());
        }

        public boolean conditionsToCheck() {
            //robotconfig.addlog(dl, "in arcTowardsBeacon", "checking against p.mm2pulses(3 * mmPerInch + 7 * mmPerInch * pi) + startEncoderPos: " + String.format(Locale.ENGLISH, "%d", p.mm2pulses(3 * mmPerInch + 7 * mmPerInch * pi) + startEncoderPos));

            if (robotconfig.debugMode) {
                if (this.isFirstTimeDebug) {
                    robotconfig.addlog(dl, "in backup42", "returning true");
                    return (true);
                } else {
                    this.isFirstTimeDebug = true;
                    robotconfig.addlog(dl, "in backup42", "returning false");
                    return (false);
                }
            } else {
                return robot.gyromoving();
            }
        }

        public void onCompletion() {
            robot.move(0, 0, 0);
        }
    };

    state backup24 = new state("backup24") {
        public void firstTime() {
            robot.setMyMotorTargets(p.mm2pulses(-24 * mmPerInch), 0, 0);
        }

        public void everyTime() {
            robot.gyromove();
            //robotconfig.addlog(dl, "in slideToTheRight", "error at " + robot.getErrors());
        }

        public boolean conditionsToCheck() {
            //robotconfig.addlog(dl, "in arcTowardsBeacon", "checking against p.mm2pulses(3 * mmPerInch + 7 * mmPerInch * pi) + startEncoderPos: " + String.format(Locale.ENGLISH, "%d", p.mm2pulses(3 * mmPerInch + 7 * mmPerInch * pi) + startEncoderPos));

            if (robotconfig.debugMode) {
                if (this.isFirstTimeDebug) {
                    robotconfig.addlog(dl, "in backup24", "returning true");
                    return (true);
                } else {
                    this.isFirstTimeDebug = true;
                    robotconfig.addlog(dl, "in backup24", "returning false");
                    return (false);
                }
            } else {
                return robot.gyromoving();
            }
        }

        public void onCompletion() {
            robot.move(0, 0, 0);
        }
    };

    state backup84 = new state("backup84") {
        public void firstTime() {
            robot.setMyMotorTargets(p.mm2pulses(-84 * mmPerInch), 0, 0);
        }

        public void everyTime() {
            robot.gyromove();
            //robotconfig.addlog(dl, "in slideToTheRight", "error at " + robot.getErrors());
        }

        public boolean conditionsToCheck() {
            return robot.gyromoving();
        }

        public void onCompletion() {
            robot.move(0, 0, 0);
        }
    };

    state driveFoward48 = new state("driveFoward48") {
        public void firstTime() {
            robot.setMyMotorTargets(p.mm2pulses(36 * mmPerInch), 0, 0);
        }

        public void everyTime() {
            robot.gyromove(1);
        }

        public boolean conditionsToCheck() {
            return robot.gyromoving();
        }

        public void onCompletion() {
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
            robot.setMyMotorTargets(0, 0, p.mm2pulses(p.spin2mm(60 * color)));
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
     * rotates robot 40 degrees clockwise if red, counter if blue
     */
    state rotate40 = new state("rotate40") {
        public void firstTime() {
            robot.setMyMotorTargets(0, 0, p.mm2pulses(p.spin2mm(40 * color)));
        }

        public void everyTime() {
            robot.bettermove();
        }

        public boolean conditionsToCheck() {
            if (robotconfig.debugMode) {
                if (this.isFirstTimeDebug) {
                    robotconfig.addlog(dl, "in rotate40", "returning true");
                    return (true);
                } else {
                    this.isFirstTimeDebug = true;
                    robotconfig.addlog(dl, "in rotate40", "returning false");
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

    state rotate110 = new state("rotate110") {
        public void firstTime() {
            robot.setMyMotorTargets(0, 0, p.mm2pulses(p.spin2mm(110 * color)));
        }

        public void everyTime() {
            robot.bettermove();
        }

        public boolean conditionsToCheck() {
            if (robotconfig.debugMode) {
                if (this.isFirstTimeDebug) {
                    robotconfig.addlog(dl, "in rotate110", "returning true");
                    return (true);
                } else {
                    this.isFirstTimeDebug = true;
                    robotconfig.addlog(dl, "in rotate110", "returning false");
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
    state rotate180 = new state("rotate180") {
        public void firstTime() {
            robot.setMyMotorTargets(0, 0, p.mm2pulses(p.spin2mm(180 * color)));
        }

        public void everyTime() {
            robot.bettermove();
        }

        public boolean conditionsToCheck() {
            if (robotconfig.debugMode) {
                if (this.isFirstTimeDebug) {
                    robotconfig.addlog(dl, "in rotate180", "returning true");
                    return (true);
                } else {
                    this.isFirstTimeDebug = true;
                    robotconfig.addlog(dl, "in rotate180", "returning false");
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

    state backuptovortexReduced = new state("backuptovortexReduced") {
        public void firstTime() {
            robot.setMyMotorTargets(p.mm2pulses(-40 * mmPerInch), 0, 0);
        }

        public void everyTime() {
            robot.bettermove(1);
        }

        public boolean conditionsToCheck() {
            if (robotconfig.debugMode) {
                if (this.isFirstTimeDebug) {
                    robotconfig.addlog(dl, "in backuptovortexReduced", "returning true");
                    return (true);
                } else {
                    this.isFirstTimeDebug = true;
                    robotconfig.addlog(dl, "in backuptovortexReduced", "returning false");
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

    state backuptovortexIncreased = new state("backuptovortexIncreased") {
        public void firstTime() {
            robot.setMyMotorTargets(p.mm2pulses(-52 * mmPerInch), 0, 0);
        }

        public void everyTime() {
            robot.bettermove(1);
        }

        public boolean conditionsToCheck() {
            if (robotconfig.debugMode) {
                if (this.isFirstTimeDebug) {
                    robotconfig.addlog(dl, "in backuptovortexIncreased", "returning true");
                    return (true);
                } else {
                    this.isFirstTimeDebug = true;
                    robotconfig.addlog(dl, "in backuptovortexIncreased", "returning false");
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

        double pulses = 2240.0;
        double endpulses = 0.0;
        double rampNumb = 3.5 / pulses;
        double puncher = 0;

        public void firstTime() {
            endpulses = robot.puncher.getCurrentPosition() + pulses;
            robot.puncher.setPower(1);
            while (!robot.garry.isPressed()) {
                Thread.yield();
            }
            robot.lvex.setPosition(1);
            robot.rvex.setPosition(1);
            robot.theHammerOfDawn.setPosition(1);
            while (robot.garry.isPressed()) {
                puncher = Math.min(1, Math.max(0.6, Math.abs(((double) robot.puncher.getCurrentPosition() - endpulses) * rampNumb)));
                robot.puncher.setPower(puncher);
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

        double pulses = 2240.0;
        double endpulses = 0.0;
        double rampNumb = 3.5 / pulses;
        double puncher = 0;

        public void firstTime() {
            endpulses = robot.puncher.getCurrentPosition() + pulses;
            robot.puncher.setPower(1);
            while (!robot.garry.isPressed()) {
                Thread.yield();
            }
            while (robot.garry.isPressed()) {
                puncher = Math.min(1, Math.max(0.6, Math.abs(((double) robot.puncher.getCurrentPosition() - endpulses) * rampNumb)));
                robot.puncher.setPower(puncher);
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
            robot.theHammerOfDawn.setPosition(0.5);
        }
    };

    state shootballOnce = new state("shoot 1 ball") {

        double pulses = 2240.0;
        double endpulses = 0.0;
        double rampNumb = 3.5 / pulses;
        double puncher = 0;

        public void firstTime() {
            endpulses = robot.puncher.getCurrentPosition() + pulses;
            robot.puncher.setPower(1);
            while (!robot.garry.isPressed()) {
                Thread.yield();
            }
            while (robot.garry.isPressed()) {
                puncher = Math.min(1, Math.max(0.6, Math.abs(((double) robot.puncher.getCurrentPosition() - endpulses) * rampNumb)));
                robot.puncher.setPower(puncher);
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

    state shootballTwoBalls = new state("shoot 2 balls") {

        double pulses = 2240.0;
        double endpulses = 0.0;
        double rampNumb = 3.5 / pulses;
        double puncher = 0;

        public void firstTime() {
            robot.theHammerOfDawn.setPosition(1);
            try {
                sleep(100);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            endpulses = robot.puncher.getCurrentPosition() + pulses;
            robot.puncher.setPower(1);
            while (!robot.garry.isPressed()) {
                Thread.yield();
            }
            robot.lvex.setPosition(1);
            robot.rvex.setPosition(1);
            while (robot.garry.isPressed()) {
                puncher = Math.min(1, Math.max(0.6, Math.abs(((double) robot.puncher.getCurrentPosition() - endpulses) * rampNumb)));
                robot.puncher.setPower(puncher);
                Thread.yield();
            }
            robot.puncher.setPower(0);
            endpulses = robot.puncher.getCurrentPosition() + pulses;
            try {
                sleep(1000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            robot.puncher.setPower(1);
            while (!robot.garry.isPressed()) {
                Thread.yield();
            }
            while (robot.garry.isPressed()) {
                puncher = Math.min(1, Math.max(0.6, Math.abs(((double) robot.puncher.getCurrentPosition() - endpulses) * rampNumb)));
                robot.puncher.setPower(puncher);
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
            robot.theHammerOfDawn.setPosition(0.5);
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
            robot.setMyMotorTargets(p.mm2pulses(12 * mmPerInch), 0, 0);
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
    state correctStrafe16 = new state("correctStrafe16") {
        public void firstTime() {
            robot.setMyMotorTargets(p.mm2pulses(16 * mmPerInch), 0, 0);
        }

        public void everyTime() {
            robot.bettermove();
        }

        public boolean conditionsToCheck() {

            //robotconfig.addlog(dl, "in backAwayFromBeacon", "checking against p.mm2pulses(3 * mmPerInch + 7 * mmPerInch * pi + 7 * mmPerInch) + startEncoderPos: " + String.format(Locale.ENGLISH, "%d", p.mm2pulses(3 * mmPerInch + 7 * mmPerInch * pi + 7 * mmPerInch) + startEncoderPos));

            if (robotconfig.debugMode) {
                if (this.isFirstTimeDebug) {
                    robotconfig.addlog(dl, "in correctStrafe16", "returning true");
                    return (true);
                } else {
                    this.isFirstTimeDebug = true;
                    robotconfig.addlog(dl, "in correctStrafe16", "returning false");
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
    state correctStrafe12 = new state("correctStrafe12") {
        public void firstTime() {
            robot.setMyMotorTargets(p.mm2pulses(12 * mmPerInch), 0, 0);
        }

        public void everyTime() {
            robot.bettermove();
        }

        public boolean conditionsToCheck() {

            //robotconfig.addlog(dl, "in backAwayFromBeacon", "checking against p.mm2pulses(3 * mmPerInch + 7 * mmPerInch * pi + 7 * mmPerInch) + startEncoderPos: " + String.format(Locale.ENGLISH, "%d", p.mm2pulses(3 * mmPerInch + 7 * mmPerInch * pi + 7 * mmPerInch) + startEncoderPos));

            if (robotconfig.debugMode) {
                if (this.isFirstTimeDebug) {
                    robotconfig.addlog(dl, "in correctStrafe12", "returning true");
                    return (true);
                } else {
                    this.isFirstTimeDebug = true;
                    robotconfig.addlog(dl, "in correctStrafe12", "returning false");
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
    state correctStrafe8 = new state("correctStrafe8") {
        public void firstTime() {
            robot.setMyMotorTargets(p.mm2pulses(8 * mmPerInch), 0, 0);
        }

        public void everyTime() {
            robot.bettermove();
        }

        public boolean conditionsToCheck() {

            //robotconfig.addlog(dl, "in backAwayFromBeacon", "checking against p.mm2pulses(3 * mmPerInch + 7 * mmPerInch * pi + 7 * mmPerInch) + startEncoderPos: " + String.format(Locale.ENGLISH, "%d", p.mm2pulses(3 * mmPerInch + 7 * mmPerInch * pi + 7 * mmPerInch) + startEncoderPos));

            if (robotconfig.debugMode) {
                if (this.isFirstTimeDebug) {
                    robotconfig.addlog(dl, "in correctStrafe8", "returning true");
                    return (true);
                } else {
                    this.isFirstTimeDebug = true;
                    robotconfig.addlog(dl, "in correctStrafe8", "returning false");
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
    state correctStrafe4 = new state("correctStrafe4") {
        public void firstTime() {
            robot.setMyMotorTargets(p.mm2pulses(4 * mmPerInch), 0, 0);
        }

        public void everyTime() {
            robot.bettermove();
        }

        public boolean conditionsToCheck() {

            //robotconfig.addlog(dl, "in backAwayFromBeacon", "checking against p.mm2pulses(3 * mmPerInch + 7 * mmPerInch * pi + 7 * mmPerInch) + startEncoderPos: " + String.format(Locale.ENGLISH, "%d", p.mm2pulses(3 * mmPerInch + 7 * mmPerInch * pi + 7 * mmPerInch) + startEncoderPos));

            if (robotconfig.debugMode) {
                if (this.isFirstTimeDebug) {
                    robotconfig.addlog(dl, "in correctStrafe4", "returning true");
                    return (true);
                } else {
                    this.isFirstTimeDebug = true;
                    robotconfig.addlog(dl, "in correctStrafe4", "returning false");
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
            robot.setMyMotorTargets(p.mm2pulses(-20 * mmPerInch), 0, 0);
        }

        public void everyTime() {
            robot.gyromove();
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
                return robot.gyromoving();
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
        }
    };

    state backAwayFromBeacon5 = new state("backAwayFromBeacon5") {
        public void firstTime() {
            robot.setMyMotorTargets(p.mm2pulses(-5 * mmPerInch), 0, 0);
        }

        public void everyTime() {
            robot.gyromove();
        }

        public boolean conditionsToCheck() {

            //robotconfig.addlog(dl, "in backAwayFromBeacon5", "checking against p.mm2pulses(3 * mmPerInch + 7 * mmPerInch * pi + 7 * mmPerInch) + startEncoderPos: " + String.format(Locale.ENGLISH, "%d", p.mm2pulses(3 * mmPerInch + 7 * mmPerInch * pi + 7 * mmPerInch) + startEncoderPos));

            if (robotconfig.debugMode) {
                if (this.isFirstTimeDebug) {
                    robotconfig.addlog(dl, "in backAwayFromBeacon5", "returning true");
                    return (true);
                } else {
                    this.isFirstTimeDebug = true;
                    robotconfig.addlog(dl, "in backAwayFromBeacon5", "returning false");
                    return (false);
                }
            } else {
                //robotconfig.addlog(dl, "in backAwayFromBeacon", "checking robot.getMotorEncoderAverage(): " + String.format(Locale.ENGLISH, "%d", robot.getMotorEncoderAverage()));
                return robot.gyromoving();
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
        }
    };

    state backAwayFromBeacon20 = new state("backAwayFromBeacon20") {
        public void firstTime() {
            robot.setMyMotorTargets(p.mm2pulses(-20 * mmPerInch), 0, 0);
        }

        public void everyTime() {
            robot.bettermove();
        }

        public boolean conditionsToCheck() {

            //robotconfig.addlog(dl, "in backAwayFromBeacon20", "checking against p.mm2pulses(3 * mmPerInch + 7 * mmPerInch * pi + 7 * mmPerInch) + startEncoderPos: " + String.format(Locale.ENGLISH, "%d", p.mm2pulses(3 * mmPerInch + 7 * mmPerInch * pi + 7 * mmPerInch) + startEncoderPos));

            if (robotconfig.debugMode) {
                if (this.isFirstTimeDebug) {
                    robotconfig.addlog(dl, "in backAwayFromBeacon20", "returning true");
                    return (true);
                } else {
                    this.isFirstTimeDebug = true;
                    robotconfig.addlog(dl, "in backAwayFromBeacon20", "returning false");
                    return (false);
                }
            } else {
                //robotconfig.addlog(dl, "in backAwayFromBeacon20", "checking robot.getMotorEncoderAverage(): " + String.format(Locale.ENGLISH, "%d", robot.getMotorEncoderAverage()));
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

    state backAwayFromBeacon15 = new state("backAwayFromBeacon15") {
        public void firstTime() {
            robot.setMyMotorTargets(p.mm2pulses(-15 * mmPerInch), 0, 0);
        }

        public void everyTime() {
            robot.bettermove();
        }

        public boolean conditionsToCheck() {

            //robotconfig.addlog(dl, "in backAwayFromBeacon15", "checking against p.mm2pulses(3 * mmPerInch + 7 * mmPerInch * pi + 7 * mmPerInch) + startEncoderPos: " + String.format(Locale.ENGLISH, "%d", p.mm2pulses(3 * mmPerInch + 7 * mmPerInch * pi + 7 * mmPerInch) + startEncoderPos));

            if (robotconfig.debugMode) {
                if (this.isFirstTimeDebug) {
                    robotconfig.addlog(dl, "in backAwayFromBeacon15", "returning true");
                    return (true);
                } else {
                    this.isFirstTimeDebug = true;
                    robotconfig.addlog(dl, "in backAwayFromBeacon15", "returning false");
                    return (false);
                }
            } else {
                //robotconfig.addlog(dl, "in backAwayFromBeaconLess", "checking robot.getMotorEncoderAverage(): " + String.format(Locale.ENGLISH, "%d", robot.getMotorEncoderAverage()));
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

    state backAwayFromBeacon10 = new state("backAwayFromBeacon10") {
        public void firstTime() {
            robot.setMyMotorTargets(p.mm2pulses(-10 * mmPerInch), 0, 0);
        }

        public void everyTime() {
            robot.bettermove();
        }

        public boolean conditionsToCheck() {

            //robotconfig.addlog(dl, "in backAwayFromBeacon10", "checking against p.mm2pulses(3 * mmPerInch + 7 * mmPerInch * pi + 7 * mmPerInch) + startEncoderPos: " + String.format(Locale.ENGLISH, "%d", p.mm2pulses(3 * mmPerInch + 7 * mmPerInch * pi + 7 * mmPerInch) + startEncoderPos));

            if (robotconfig.debugMode) {
                if (this.isFirstTimeDebug) {
                    robotconfig.addlog(dl, "in backAwayFromBeacon10", "returning true");
                    return (true);
                } else {
                    this.isFirstTimeDebug = true;
                    robotconfig.addlog(dl, "in backAwayFromBeacon10", "returning false");
                    return (false);
                }
            } else {
                //robotconfig.addlog(dl, "in backAwayFromBeacon10", "checking robot.getMotorEncoderAverage(): " + String.format(Locale.ENGLISH, "%d", robot.getMotorEncoderAverage()));
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
