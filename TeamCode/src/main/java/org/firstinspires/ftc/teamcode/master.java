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

@Autonomous(name = "anything", group = "above")

public class master extends LinearOpMode {
    //import library for precise movement
    private preciseMovement p = new preciseMovement();
    //import file of states
    private stateslist states = new stateslist();
    //make list of states to choose from
    private List<state> chooselist = Arrays.asList(states.sleep500, states.sleep2000, states.sleep10000, states.rotate40, states.rotate60, states.rotate90, states.rotate180, states.backup24, states.backup30, states.backup84, states.slideToTheLeft, states.slideToTheRight50, states.slideToTheRight54, states.slideToTheRight58, states.driveTowardsBeacon, states.backuptovortexReduced, states.backuptovortex, states.backuptovortexIncreased, states.backAwayFromBeacon10, states.backAwayFromBeacon15, states.backAwayFromBeacon20, states.shootball, states.shootball2, states.pushBeaconButton, states.arcTowardsBeacon20, states.arcTowardsBeacon24, states.arcTowardsBeacon28, states.arcTowardsBeacon32, states.pivotbeaconless, states.pivotbeacon, states.scanForLine, states.noscope, states.correctStrafe4, states.correctStrafe8, states.correctStrafe12, states.correctStrafe16, states.colorBlue, states.colorRed);
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
        //add state to set color to red as the first of many default states
        list.add(0, states.colorRed);
//        //add state to arc towards the beacon with an inner radius of 28 inches
//        list.add(1, states.arcTowardsBeacon28);
//        //add state to scan for the white tape line while straffing right, stopping on the line
//        list.add(2, states.scanForLine);
//        //add state to drive forwards until robot touches beacon
//        list.add(3, states.driveTowardsBeacon);
//        //add state to use color sensor and push corresponding button with the button pusher servo
//        list.add(4, states.pushBeaconButton);
//        //add state to back away from beacon 20 inches to prepare for shooting ball
//        list.add(5, states.backAwayFromBeacon20);
//        //add state to spin launcher motor one complete cycle, also activates primary intake
//        list.add(6, states.shootball);
//        //add state to wait 2 seconds for second ball to intake
//        list.add(7, states.sleep2000);
//        //add state to spin launcher motor one complete cycle and stops primary intake
//        list.add(8, states.shootball2);
//        //add state to drive back foward 12 inches after shooting ball to get within range of the tape
//        list.add(9, states.correctStrafe12);
//        //add state to strafe in the direction of the second beacon 54 inches at a faster speed then when scanning for the line
//        list.add(10, states.slideToTheRight54);
//        //same as previous scanForLine state
//        list.add(11, states.scanForLine);
//        //same as previous driveTowardsBeacon state
//        list.add(12, states.driveTowardsBeacon);
//        //same as previous pushBeaconButton state
//        list.add(13, states.pushBeaconButton);
//        //add state to pivot robot backwards towards center vortex
//        list.add(14, states.pivotbeacon);
//        //add state to backup on to center vortex to knock off cap ball and park
//        list.add(15, states.backuptovortex);
        //loop to use joystick to edit list of states
        while (selecting) {
            //loop to add each element in list of current states to telemetry data
            for (int index = 0; index < list.size(); index++) {
                state currentState = list.get(index);
                //check if the current state is selected
                if (index == selectedstate)
                    //add visual indicator to name of state
                    telemetry.addData(String.valueOf(index), "_" + currentState.name + "_");
                else
                    //just display name of state normally
                    telemetry.addData(String.valueOf(index), currentState.name);
            }
            //display telementry data
            telemetry.update();
            //set loop variable to true for exiting the loop
            boolean loop = true;
            //loop continuously while checking the buttons until one is pressed
            while (loop) {
                if (gamepad1.dpad_up) {
                    //loop while button is held to prevent double presses
                    while (gamepad1.dpad_up) {
                        idle();
                    }
                    //stop button checking loop after current pass
                    loop = false;
                    //select previous state
                    selectedstate--;
                    //add size of list to selected state to avoid negative state number
                    selectedstate = list.size() + selectedstate;
                    //set the selectedstate to the remainder of the selected state divided by the list size so the selection can rollover at the beginning and end of the list
                    selectedstate = selectedstate % list.size();
                }
                if (gamepad1.dpad_down) {
                    //loop while button is held to prevent double presses
                    while (gamepad1.dpad_down) {
                        idle();
                    }
                    //stop button checking loop after current pass
                    loop = false;
                    //select next state
                    selectedstate++;
                    //set the selectedstate to the remainder of the selected state divided by the list size so the selection can rollover at the beginning and end of the list
                    selectedstate = selectedstate % list.size();
                }
                if (gamepad1.a) {
                    //loop while button is held to prevent double presses
                    while (gamepad1.a) {
                        idle();
                    }
                    //stop button checking loop after current pass
                    loop = false;
                    //get index in chooselist of the selected state, verifying that it exist in the chooselist
                    int choiceindex = Math.max(chooselist.indexOf(list.get(selectedstate)), 0);
                    //add copy of current state after the current state
                    list.add(selectedstate + 1, chooselist.get(choiceindex));
                    //select the added state
                    selectedstate++;
                }
                if (gamepad1.b) {
                    //loop while button is held to prevent double presses
                    while (gamepad1.b) {
                        idle();
                    }
                    //stop button checking loop after current pass
                    loop = false;
                    //delete selected state
                    list.remove(selectedstate);
                    selectedstate++;
                    selectedstate = list.size() + selectedstate;
                    selectedstate = selectedstate % list.size();
                }
                if (gamepad1.start) {
                    //loop while button is held to prevent double presses
                    while (gamepad1.start) {
                        idle();
                    }
                    //stop button checking loop after current pass
                    loop = false;
                    //also stop program state selection loop
                    selecting = false;
                }
                if (gamepad1.dpad_left) {
                    //loop while button is held to prevent double presses
                    while (gamepad1.dpad_left) {
                        idle();
                    }
                    //stop button checking loop after current pass
                    loop = false;
                    //get index of previous item in chooselist of the selected state
                    int choiceindex = (chooselist.size() + chooselist.indexOf(list.get(selectedstate)) - 1) % chooselist.size();
                    //delete selected state
                    list.remove(selectedstate);
                    //replace selected state with the item before it in the choice list
                    list.add(selectedstate, chooselist.get(choiceindex));
                    //make sure index of selected state is within range of the list
                    selectedstate = Math.min(Math.max(selectedstate, 0), list.size() - 1);
                }
                if (gamepad1.dpad_right) {
                    //loop while button is held to prevent double presses
                    while (gamepad1.dpad_right) {
                        idle();
                    }
                    //stop button checking loop after current pass
                    loop = false;
                    //get index of next item in chooselist of the selected state
                    int choiceindex = (chooselist.size() + chooselist.indexOf(list.get(selectedstate)) + 1) % chooselist.size();
                    //delete selected state
                    list.remove(selectedstate);
                    //replace selected state with the item after it in the choice list
                    list.add(selectedstate, chooselist.get(choiceindex));
                    //make sure index of selected state is within range of the list
                    selectedstate = Math.min(Math.max(selectedstate, 0), list.size() - 1);
                }
            }
        }
        // send whole LinearOpMode object and context to robotconfig init method
        robot.init(this);
        //add telementry to report that init completed
        robotconfig.addlog(dl, "autonomous", "Done with robot.init --- waiting for start in " + this.getClass().getSimpleName());
        //display list of states that will be run
        for (int index = 0; index < list.size(); index++) {
            state currentState = list.get(index);
            telemetry.addData(String.valueOf(index), currentState.name);
        }
        //convert list of states to be run to array for theoretical performance reasons
        runlist = list.toArray(new state[list.size()]);
        //run each state multiple times until the state increases the currentState variable by 1
        currentState = 0;
        //default current color to purple, first state will either redefine it as red or blue
        states.color = 0;
        //add telementry data to display if debug mode is active, debug mode is used to test to make sure objects are oriented correctly without having actual hardware attached
        telemetry.addData("Say", "Hello Driver - debug mode is " + robotconfig.debugMode);
        //display telementry data
        telemetry.update();
        waitForStart();
        //add log to log file
        robotconfig.addlog(dl, "autonomous", "Started");
        //loop while match is running
        while (opModeIsActive()) {
            
            robotconfig.addlog(dl, "Mainline", "Beginning state machine pass " + String.format(Locale.ENGLISH, "%d", currentState));
            
            //check if the currentState is more than the last index of the runlist
            if (currentState < runlist.length) {
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

}
