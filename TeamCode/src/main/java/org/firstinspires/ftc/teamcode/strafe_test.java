package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Func;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

import static org.firstinspires.ftc.teamcode.robotconfig.dl;
import static org.firstinspires.ftc.teamcode.stateslist.currentState;
import static org.firstinspires.ftc.teamcode.stateslist.robot;

/**
 * Autonomous that can do anything
 */

@Autonomous(name = "Jones", group = "above")

public class strafe_test extends LinearOpMode {
    //import library for precise movement
    private preciseMovement p = new preciseMovement();
    //import file of states
    private stateslist states = new stateslist();
    //make list of the states that are going to be run
    private List<state> list = new ArrayList<>();
    //init the array that is going to be used when actually running
    private state[] runlist;

    @Override
    public void runOpMode() {
        //start with the first state selected
        int selectedstate = 0;
        //set variable selecting to true for the selecting while loop
        boolean selecting = true;

        askState(states.colorRed, states.colorBlue);
        askState(states.arcTowardsBeacon);
        askState(states.scanForLine);
        askState(states.driveTowardsBeacon);
        askState(states.pushBeaconButton);
        askState(states.backAwayFromBeacon5);
        for (int i = 0; i < 20; i++) {
            askState(states.slideToTheRight);
            askState(states.scanForLine);
            askState(states.sleep2000);
//        askState(states.driveTowardsBeacon);
//        askState(states.pushBeaconButton);
//        askState(states.backAwayFromBeacon5);
            askState(states.slideToTheLeft);
            askState(states.scanForLineInverted);
            askState(states.sleep2000);
//        askState(states.driveTowardsBeacon);
//        askState(states.pushBeaconButton);
        }

        askState(states.sleep0);

        if (ask("Ready to init?", "Ready to init? asked again so I don't need to make a new method")) {

        }

        // send whole LinearOpMode object and context to robotconfig init method
        robot.init(this);
        //add telementry to report that init completed
        robotconfig.addlog(dl, "autonomous", "Done with robot.init --- waiting for start in " + this.getClass().getSimpleName());

        //convert list of states to be run to array for theoretical performance reasons
        runlist = list.toArray(new state[list.size()]);
        //run each state multiple times until the state increases the currentState variable by 1
        currentState = 0;
        //default current color to purple, first state will either redefine it as red or blue
        states.color = 0;
        //add telementry data to display if debug mode is active, debug mode is used to test to make sure objects are oriented correctly without having actual hardware attached
        telemetry.addData("Say", "Hello Driver - debug mode is " + robotconfig.debugMode);
        displayStates();
        //display telementry data
        telemetry.update();
        telemetry.setMsTransmissionInterval(20);
        waitForStart();
        telemetry.addLine()
                .addData("state", new Func<String>() {
                    @Override
                    public String value() {
                        return runlist[currentState].name;
                    }
                });
//        telemetry.addLine()
//                .addData("gyroP", new Func<String>() {
//                    @Override
//                    public String value() {
//                        return String.format(Locale.ENGLISH, "%.2f", robot.gyroPid.getOutput());
//                    }
//                })
//                .addData("ultraP", new Func<String>() {
//                    @Override
//                    public String value() {
//                        return String.format(Locale.ENGLISH, "%.2f", robot.ultraPid.getOutput());
//                    }
//                });
        //add log to log file
        robotconfig.addlog(dl, "autonomous", "Started");
        //loop while match is running
        while (opModeIsActive()) {

            robotconfig.addlog(dl, "Mainline", "Beginning state machine pass " + String.format(Locale.ENGLISH, "%d", currentState));
            //check if the currentState is more than the last index of the runlist
            if (currentState + 1 < runlist.length) {
                telemetry.update();
                //run the state from the runlist of the currentState index
                runlist[currentState].run();
                //add log of name of state that ran for debugging
                robotconfig.addlog(dl, "Mainline", "Ending state machine pass of " + runlist[currentState].name);
            } else {
                //if completed last state, stop
                robot.move(0, 0, 0);
                //add log to tell us that the program finished all selected states before stopping
                robotconfig.addlog(dl, "StateMachine", "stop requested");
                requestOpModeStop();
            }

        }

        robot.move(0, 0, 0);
        robot.pushButton(0);
        //add log to tell us that the program stopped smoothly
        robotconfig.addlog(dl, "autonomous", "Done with opmode, exited based on OpmodeIsActive false");

    }

    void displayStates() {
        telemetry.addLine();
        //display list of states that will be run
        for (int index = 0; index < list.size(); index++) {
            state currentState = list.get(index);
            telemetry.addData(String.valueOf(index), currentState.name);
        }
    }

    void askState(state statea) {
        list.add(list.size(), statea);
    }

    void askState(state statea, state stateb) {
        list.add(list.size(), ask(statea, stateb));
    }

    void askState(state statea, state stateb, state statec) {
        list.add(list.size(), ask(statea, stateb, statec));
    }

    void askState(state statea, state stateb, state statex, state statey) {
        list.add(list.size(), ask(statea, stateb, statex, statey));
    }

    boolean ask(String statea, String stateb) {
        telemetry.addData("A", statea);
        telemetry.addData("B", stateb);
        displayStates();
        telemetry.update();

        while (!isStopRequested() && !opModeIsActive()) {
            if (gamepad1.a) {
                while (gamepad1.a)
                    idle();
                return true;
            } else if (gamepad1.b) {
                while (gamepad1.b)
                    idle();
                return false;
            }
        }

        return true;
    }

    state ask(state statea, state stateb) {
        telemetry.addData("A", statea.name);
        telemetry.addData("B", stateb.name);
        displayStates();
        telemetry.update();
        while (!isStopRequested() && !opModeIsActive()) {
            if (gamepad1.a) {
                while (gamepad1.a)
                    idle();
                return statea;
            } else if (gamepad1.b) {
                while (gamepad1.b)
                    idle();
                return stateb;
            }
        }
        return statea;
    }

    state ask(state statea, state stateb, state statex) {
        telemetry.addData("A", statea.name);
        telemetry.addData("B", stateb.name);
        telemetry.addData("X", statex.name);
        displayStates();
        telemetry.update();
        while (!isStopRequested() && !opModeIsActive()) {
            if (gamepad1.a) {
                while (gamepad1.a)
                    idle();
                return statea;
            } else if (gamepad1.b) {
                while (gamepad1.b)
                    idle();
                return stateb;
            } else if (gamepad1.x) {
                while (gamepad1.x)
                    idle();
                return statex;
            }
        }
        return statea;
    }

    state ask(state statea, state stateb, state statex, state statey) {
        telemetry.addData("A", statea.name);
        telemetry.addData("B", stateb.name);
        telemetry.addData("X", statex.name);
        telemetry.addData("Y", statey.name);
        displayStates();
        telemetry.update();
        while (!isStopRequested() && !opModeIsActive()) {
            if (gamepad1.a) {
                while (gamepad1.a)
                    idle();
                return statea;
            } else if (gamepad1.b) {
                while (gamepad1.b)
                    idle();
                return stateb;
            } else if (gamepad1.x) {
                while (gamepad1.x)
                    idle();
                return statex;
            } else if (gamepad1.y) {
                while (gamepad1.y)
                    idle();
                return statey;
            }
        }
        return statea;
    }

}
