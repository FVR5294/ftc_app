package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Iterator;
import java.util.List;
import java.util.Locale;

import static org.firstinspires.ftc.teamcode.robotconfig.dl;
import static org.firstinspires.ftc.teamcode.stateslist.currentState;
import static org.firstinspires.ftc.teamcode.stateslist.robot;

/**
 * Autonomous for red side that shoots 2 balls
 */

@Autonomous(name = "master", group = "above")

public class master extends LinearOpMode {
    private preciseMovement p = new preciseMovement();
    private stateslist states = new stateslist();
    private List<state> chooselist = Arrays.asList(states.rotate40, states.rotate40, states.rotate60, states.rotate90, states.rotate180, states.backup24, states.backup30, states.slideToTheLeft, states.slideToTheRight, states.backuptovortex, states.backAwayFromBeacon, states.shootball, states.shootball2, states.pushBeaconButton, states.arcTowardsBeacon, states.pivotbeacon, states.noscope, states.correctStrafe, states.colorBlue, states.colorRed);
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
        while (selecting) {
            for (Iterator<state> iterator = list.iterator(); iterator.hasNext(); ) {
                state currentState = iterator.next();
                int index = list.indexOf(currentState);
                if (index == selectedstate)
                    telemetry.addData(String.valueOf(index), "_" + currentState.name + "_");
                else
                    telemetry.addData(String.valueOf(index), currentState.name);
            }
            telemetry.update();
            sleep(500);
            boolean loop = true;
            while (loop) {
                if (gamepad1.dpad_up) {
                    loop = false;
                    selectedstate -= 1;
                    selectedstate = +selectedstate;
                    selectedstate = selectedstate % list.size();
                }
                if (gamepad1.dpad_down) {
                    loop = false;
                    selectedstate += 1;
                    selectedstate = +selectedstate;
                    selectedstate = selectedstate % list.size();
                }
                if (gamepad1.a) {
                    loop = false;
                    int choiceindex = (chooselist.size() + chooselist.indexOf(list.get(selectedstate)) - 1) % chooselist.size();
                    list.add(selectedstate, chooselist.get(choiceindex));
                    selectedstate = +selectedstate;
                    selectedstate = selectedstate % list.size();
                }
                if (gamepad1.b) {
                    loop = false;
                    list.remove(selectedstate);
                    selectedstate = +selectedstate;
                    selectedstate = selectedstate % list.size();
                }
                if (gamepad1.start) {
                    loop = false;
                    selecting = false;
                }
                if (gamepad1.dpad_left) {
                    loop = false;
                    int choiceindex = (chooselist.size() + chooselist.indexOf(list.get(selectedstate)) - 1) % chooselist.size();
                    list.remove(selectedstate);
                    list.add(selectedstate, chooselist.get(choiceindex));
                    selectedstate = +selectedstate;
                    selectedstate = selectedstate % list.size();
                }
                if (gamepad1.dpad_right) {
                    loop = false;
                    int choiceindex = (chooselist.size() + chooselist.indexOf(list.get(selectedstate)) + 1) % chooselist.size();
                    list.remove(selectedstate);
                    list.add(selectedstate, chooselist.get(choiceindex));
                    selectedstate = +selectedstate;
                    selectedstate = selectedstate % list.size();
                }
            }
        }
        for (Iterator<state> iterator = list.iterator(); iterator.hasNext(); ) {
            state currentState = iterator.next();
            int index = list.indexOf(currentState);
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

            if (currentState < runlist.length) {
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
