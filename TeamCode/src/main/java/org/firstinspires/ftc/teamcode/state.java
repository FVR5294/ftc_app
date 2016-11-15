package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.robotconfig.dl;

/**
 * Created by mail2 on 11/15/2016.
 */

interface substate {
    void firstTime();

    void everyTime();

    boolean conditionsToCheck();

    void onCompletion();
}

class state implements substate {
    public String name;
    boolean firstTime = true;

    public state(String name) {
        this.name = name;
    }

    public void firstTime() {

    }

    public void everyTime() {

    }

    public boolean conditionsToCheck() {
        return true;
    }

    public void onCompletion() {

    }

    public void run() {
        robotconfig.addlog(dl, "error", "state not defined");

    }
}
