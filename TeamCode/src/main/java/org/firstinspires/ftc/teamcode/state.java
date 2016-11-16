package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.robotconfig.dl;
import static org.firstinspires.ftc.teamcode.stateslist.currentState;

/**
 * Created by mail2 on 11/15/2016.
 * Project: ftc_app_for_2016_robot
 */

interface substate {
    void firstTime();

    void everyTime();

    boolean conditionsToCheck();

    void onCompletion();
}

class state implements substate {
    public String name;
    private boolean isFirstTime = true;

    public state(String name) {
        this.name = name;
    }

    /***
     * method to be run only the first iteration
     */
    public void firstTime() {

    }

    /***
     * method to be run every time including the first iteration
     */
    public void everyTime() {

    }

    /***
     * check if exit conditions are met every iteration
     *
     * @return true if should exit
     */
    public boolean conditionsToCheck() {
        return true;
    }

    /***
     * method to be run at the end of the last iteration
     * switch states is embedded in the run method
     */
    public void onCompletion() {

    }

    /***
     * method uses previously defined methods to make a method to be run every iteration of the state machine for a specific state
     */
    public void run() {
        if (this.isFirstTime) {
            this.firstTime();
            robotconfig.addlog(dl, "StateMachine", "Execution of " + this.name + " has started");
            this.isFirstTime = false;
        }
        everyTime();
        if (conditionsToCheck()) {
            this.onCompletion();
            currentState++;
            robotconfig.addlog(dl, "StateMachine", "Execution of " + this.name + " has been completed");
        }
    }
}
