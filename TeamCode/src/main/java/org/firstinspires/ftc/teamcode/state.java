package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Locale;

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
    public boolean isFirstTimeDebug = false;
    public int runCount = 0;
    public ElapsedTime runtimecounter = new ElapsedTime();
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
            robotconfig.addlog(dl, this.name + "StateMachine", "Execution of " + this.name + " has started");
            this.isFirstTime = false;
            this.runtimecounter.reset();
            this.runCount = 0;
            //robotconfig.addlog(dl, this.name + "StateMachine", "after this.isFirstTime = false");
        }
        //robotconfig.addlog(dl, this.name + "StateMachine", "before if(conditionsToCheck");
        if (this.conditionsToCheck()) {
            //robotconfig.addlog(dl, this.name + "StateMachine", "before this.onCompletion");
            this.onCompletion();
            //robotconfig.addlog(dl, this.name + "StateMachine", "after this.onCompletion");
            currentState++;
            this.isFirstTime = true;
            //robotconfig.addlog(dl, this.name + "StateMachine", "currentState++");
            robotconfig.addlog(dl, this.name + "StateMachine", String.format(Locale.ENGLISH, "Execution of %s has been completed in, %d, intervals over a time of, %f.3, seconds", this.name, this.runCount, this.runtimecounter.seconds()));
        } else {
            //robotconfig.addlog(dl, this.name + "StateMachine", "before this.everyTime");
            everyTime();
            this.runCount++;
            //robotconfig.addlog(dl, this.name + "StateMachine", "after this.everyTime");
        }
        //robotconfig.addlog(dl, this.name + "StateMachine", "run completed");
    }
}
