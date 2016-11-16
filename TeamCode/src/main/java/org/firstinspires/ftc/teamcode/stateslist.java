package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import static org.firstinspires.ftc.teamcode.robotconfig.dl;

/**
 * Created by mail2 on 11/15/2016.
 */

public class stateslist {

    public static robotconfig robot;
    public static int currentState;
    public int color = 0;
    /***
     * state makes robot drive forward slightly
     */
    public state clearWall = new state("clear el wall") {
        public void firstTime() {
            robot.move(1, 0, 0);
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
     * state makes robot arc 90 degrees so it ends up pointed towards the beacon
     */
    public state arcTorwardsBeacon = new state("arc twards beacon") {
        public void firstTime() {

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
     * state makes robot drive closer to the wall so the sensor is in range of the tape
     */
    public state getCloserToWall = new state("move robot slightly closer to wall") {
        public void firstTime() {

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
     * state uses the light sensor to strafe towards the tape line
     */
    public state scanForLine = new state("use light sensor to move twards line") {
        public void firstTime() {

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
     * state makes robot drive forward until touch sensor is touching beacon
     */
    public state driveTowardsBeacon = new state("stab beacon with touch sensor") {
        public void firstTime() {

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
     * state makes robot use color sensor and servo to try to press the button on the beacon
     */
    public state pushBeaconButton = new state("use servo to select color") {
        public void firstTime() {

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
    public state backAwayFromBeacon = new state("back away from the beacon") {
        public void firstTime() {

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
     * state makes robot strafe towards the next beacon
     */
    public state driveToNextBeacon = new state("drive twards second beacon") {
        public void firstTime() {

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
     * state makes the robot attempt to knock over the capt ball
     */
    public state retreatToCenter = new state("drive twards the center goal") {
        public void firstTime() {

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
     * state makes robot move backwards to try to park partially on the center vortex
     */
    public state driveOnToWood = new state("drive backwards onto the wood of the center goal") {
        public void firstTime() {

        }

        public void everyTime() {

        }

        public boolean conditionsToCheck() {
            return true;
        }

        public void onCompletion() {

        }
    };

    public stateslist(LinearOpMode op, int color) {
        this.color = color;
        if (color == 1) {
            robotconfig.addlog(dl, "stateslist", "selected color red");
            robot = refactoredRedAutonomous.redRobot;
            currentState = refactoredRedAutonomous.currentRedState;
        } else if (color == -1) {
            robotconfig.addlog(dl, "stateslist", "selected color blue");
            robot = refactoredBlueAutonomous.blueRobot;
            currentState = refactoredBlueAutonomous.currentBlueState;
        } else {
            robotconfig.addlog(dl, "error", "color is undefined");
        }
    }
}
