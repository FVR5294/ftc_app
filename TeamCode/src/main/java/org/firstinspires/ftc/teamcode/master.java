package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Locale;

import static org.firstinspires.ftc.teamcode.robotconfig.dl;
import static org.firstinspires.ftc.teamcode.stateslist.currentState;
import static org.firstinspires.ftc.teamcode.stateslist.robot;

/**
 * Autonomous that can do anything
 */

@Autonomous(name = "master", group = "above")

public class master extends LinearOpMode {
    private preciseMovement p = new preciseMovement();
    private stateslist states = new stateslist();
    private List<state> chooselist = Arrays.asList(states.sleep500, states.sleep2000, states.sleep10000, states.rotate40, states.rotate40, states.rotate60, states.rotate90, states.rotate180, states.backup24, states.backup30, states.backup84, states.slideToTheLeft, states.slideToTheRight, states.driveTowardsBeacon, states.backuptovortex, states.backAwayFromBeacon, states.shootball, states.shootball2, states.pushBeaconButton, states.arcTowardsBeacon, states.pivotbeacon, states.pivotbeacon2, states.scanForLine, states.noscope, states.correctStrafe, states.colorBlue, states.colorRed);
    private List<state> list = new ArrayList<state>();
    private state[] runlist;

    @Override
    public void runOpMode() {
        robot.init(this);  // send whole LinearOpMode object and context
        robotconfig.addlog(dl, "autonomous", "Done with robot.init --- waiting for start in " + this.getClass().getSimpleName());
        states.color = 0;//tell the state list what the current color is
        telemetry.addData("Say", "Hello Driver - debug mode is " + robotconfig.debugMode);
        telemetry.update();
        int selectedstate = 0;
        boolean selecting = true;
        list.add(selectedstate, chooselist.get(0));
        while (selecting) {
            for (int index = 0; index < list.size(); index++) {
                state currentState = list.get(index);
                if (index == selectedstate)
                    telemetry.addData(String.valueOf(index), "_" + currentState.name + "_");
                else
                    telemetry.addData(String.valueOf(index), currentState.name);
            }
            telemetry.update();
            boolean loop = true;
            while (loop) {
                if (gamepad1.dpad_up) {
                    while (gamepad1.dpad_up) {
                        idle();
                    }
                    loop = false;
                    selectedstate--;
                    selectedstate = list.size() + selectedstate;
                    selectedstate = selectedstate % list.size();
                }
                if (gamepad1.dpad_down) {
                    while (gamepad1.dpad_down) {
                        idle();
                    }
                    loop = false;
                    selectedstate++;
                    selectedstate = list.size() + selectedstate;
                    selectedstate = selectedstate % list.size();
                }
                if (gamepad1.a) {
                    while (gamepad1.a) {
                        idle();
                    }
                    loop = false;
                    int choiceindex = (chooselist.size() + chooselist.indexOf(list.get(selectedstate)) - 1) % chooselist.size();
                    selectedstate++;
                    list.add(selectedstate, chooselist.get(choiceindex));
                }
                if (gamepad1.b) {
                    while (gamepad1.b) {
                        idle();
                    }
                    loop = false;
                    list.remove(selectedstate);
                    selectedstate++;
                    selectedstate = list.size() + selectedstate;
                    selectedstate = selectedstate % list.size();
                }
                if (gamepad1.start) {
                    while (gamepad1.start) {
                        idle();
                    }
                    loop = false;
                    selecting = false;
                }
                if (gamepad1.dpad_left) {
                    while (gamepad1.dpad_left) {
                        idle();
                    }
                    loop = false;
                    int choiceindex = (chooselist.size() + chooselist.indexOf(list.get(selectedstate)) - 1) % chooselist.size();
                    list.remove(selectedstate);
                    list.add(selectedstate, chooselist.get(choiceindex));
                    selectedstate = list.size() + selectedstate;
                    selectedstate = selectedstate % list.size();
                }
                if (gamepad1.dpad_right) {
                    while (gamepad1.dpad_right) {
                        idle();
                    }
                    loop = false;
                    int choiceindex = (chooselist.size() + chooselist.indexOf(list.get(selectedstate)) + 1) % chooselist.size();
                    list.remove(selectedstate);
                    list.add(selectedstate, chooselist.get(choiceindex));
                    selectedstate = list.size() + selectedstate;
                    selectedstate = selectedstate % list.size();
                }
            }
        }
        for (int index = 0; index < list.size(); index++) {
            state currentState = list.get(index);
            if (index == selectedstate)
                telemetry.addData(String.valueOf(index), "_" + currentState.name + "_");
            else
                telemetry.addData(String.valueOf(index), currentState.name);
        }
        runlist = list.toArray(new state[list.size()]);
        currentState = 0;//run each state multiple times until the state increases the currentState variable by 1
        telemetry.addData("Say", "Hello Driver - debug mode is " + robotconfig.debugMode);
        telemetry.update();
        waitForStart();
        robotconfig.addlog(dl, "autonomous", "Started");

        while (opModeIsActive()) {

            robotconfig.addlog(dl, "Mainline", "Beginning state machine pass " + String.format(Locale.ENGLISH, "%d", currentState));

            if (currentState < runlist.length - 1) {
                runlist[currentState].run();
            } else {
                robot.move(0, 0, 0);
                robotconfig.addlog(dl, "StateMachine", "stop requested");
                requestOpModeStop();
            }

            robotconfig.addlog(dl, "Mainline", "Ending state machine pass of " + runlist[currentState].name);

        }

        robot.move(0, 0, 0);
        robot.pushButton(0);

        robotconfig.addlog(dl, "autonomous", "Done with opmode, exited based on OpmodeIsActive false");

    }

}
