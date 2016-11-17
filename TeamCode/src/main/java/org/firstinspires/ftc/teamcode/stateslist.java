package org.firstinspires.ftc.teamcode;

import static java.lang.Thread.sleep;
import static org.firstinspires.ftc.teamcode.measurements.mmPerInch;
import static org.firstinspires.ftc.teamcode.measurements.pi;

/**
 * Created by mail2 on 11/15/2016.
 * Project: ftc_app_for_2016_robot
 */

class stateslist {

    public static robotconfig robot = new robotconfig();
    static int currentState;
    public int color = 0;
    public preciseMovement p = new preciseMovement();
    public int startEncoderPos = 0;

    /***
     * state makes robot drive forward slightly
     */
    state clearWall = new state("clear el wall") {

        public void firstTime() {
            robot.move(1, 0, 0);
        }

        public void everyTime() {

        }

        public boolean conditionsToCheck() {
            return robot.getMotorEncoderAverage() > p.mm2pulses(3 * mmPerInch) + startEncoderPos;
        }

        public void onCompletion() {

        }
    };

    /***
     * state makes robot arc 90 degrees so it ends up pointed towards the beacon
     */
    state arcTorwardsBeacon = new state("arc twards beacon") {


        public void firstTime() {
            //middle distance is mmPerInch*28*pi/4
            //inside distance is mmPerInch*21*pi/4
            //outside distance is mmPerInch*37*pi/4
            //mmPerInch*28*pi/4 + mmPerInch*9*pi/4 = 1
            robot.move((28 / (9 + 28)), 0, -color * (9 / (9 + 28)));
        }

        public void everyTime() {

        }

        public boolean conditionsToCheck() {
            return robot.getMotorEncoderAverage() > p.mm2pulses(3 * mmPerInch + 7 * mmPerInch * pi) + startEncoderPos;
        }

        public void onCompletion() {
            robot.enableEncodersToPosition();
            p.automaticSquareUp(robot);
            robot.enableMotorEncoders();
        }
    };

    /***
     * state makes robot drive closer to the wall so the sensor is in range of the tape
     */
    state getCloserToWall = new state("move robot slightly closer to wall") {
        public void firstTime() {
            robot.move(1, 0, 0);
        }

        public void everyTime() {

        }

        public boolean conditionsToCheck() {
            return robot.getMotorEncoderAverage() > p.mm2pulses(3 * mmPerInch + 7 * mmPerInch * pi + 7 * mmPerInch) + startEncoderPos;
        }

        public void onCompletion() {

        }
    };

    /***
     * state uses the light sensor to strafe towards the tape line
     */
    state scanForLine = new state("use light sensor to move twards line") {
        public void firstTime() {
            robot.enableMotorBreak();
            robot.move(0, color, 0);
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
    state driveTowardsBeacon = new state("stab beacon with touch sensor") {
        public void firstTime() {
            robot.disableMotorBreak();
            robot.move(0.5, 0, 0);
        }

        public void everyTime() {

        }

        public boolean conditionsToCheck() {
            return robot.touchBeacon.isPressed();
        }

        public void onCompletion() {

        }
    };
    /***
     * state makes robot use color sensor and servo to try to press the button on the beacon
     */
    state pushBeaconButton = new state("use servo to select color") {
        public void firstTime() {
            robot.pushButton(robot.detectColor() * color);
            try {
                sleep(100);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }

        public void everyTime() {

        }

        public boolean conditionsToCheck() {
            return true;
        }

        public void onCompletion() {

        }
    };
    /***
     * state makes robot back away from beacon slightly to avoid running into anything during next state
     */
    state backAwayFromBeacon = new state("back away from the beacon") {
        public void firstTime() {
            robot.enableMotorBreak();
            robot.move(-1, 0, 0);
            startEncoderPos = robot.getMotorEncoderAverage();
        }

        public void everyTime() {

        }

        public boolean conditionsToCheck() {
            return robot.getMotorEncoderAverage() < -p.mm2pulses(3 * mmPerInch);
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

    /***
     * state makes the robot attempt to knock over the capt ball
     */
    state retreatToCenter = new state("drive twards the center goal") {

        public void firstTime() {
            robot.disableMotorBreak();
            startEncoderPos = robot.getMotorEncoderAverage();
            robot.move(-1, -color, 0);
        }

        public void everyTime() {

        }

        public boolean conditionsToCheck() {
            return (robot.getMotorEncoderAverage() - startEncoderPos) * 2 < -p.mm2pulses(55 * mmPerInch);
        }

        public void onCompletion() {

        }
    };
    /***
     * state makes robot move backwards to try to park partially on the center vortex
     */
    state driveOnToWood = new state("drive backwards onto the wood of the center goal") {
        public void firstTime() {
            robot.enableMotorBreak();
            robot.move(-1, 0, 0);
            startEncoderPos = robot.getMotorEncoderAverage();
        }

        public void everyTime() {

        }

        public boolean conditionsToCheck() {
            return (robot.getMotorEncoderAverage() - startEncoderPos) * 2 < -p.mm2pulses(4 * mmPerInch);
        }

        public void onCompletion() {
            robot.move(0, 0, 0);
        }
    };
}
