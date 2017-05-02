package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import static org.firstinspires.ftc.teamcode.robotconfig.dl;
import static org.firstinspires.ftc.teamcode.stateslist.currentState;
import static org.firstinspires.ftc.teamcode.stateslist.robot;

/**
 * Autonomous for red side that shoots 2 balls
 */

//@Autonomous(name = "double red", group = "red")

public class double_red extends LinearOpMode {
    private preciseMovement p = new preciseMovement();
    private stateslist state = new stateslist();

    @Override
    public void runOpMode() {
        robot.init(this);  // send whole LinearOpMode object and context
        robotconfig.addlog(dl, "autonomous", "Done with robot.init --- waiting for start in " + this.getClass().getSimpleName());
        state.color = 1;//tell the state list what the current color is
        currentState = 0;//run each state multiple times until the state increases the currentState variable by 1
        telemetry.addData("Say", "Hello Driver - debug mode is " + robotconfig.debugMode);
        telemetry.update();
        waitForStart();
        robotconfig.addlog(dl, "autonomous", "Started");

        while (opModeIsActive()) {

            //robotconfig.addlog(dl, "Mainline", "Beginning state machine pass " + String.format(Locale.ENGLISH, "%d", currentState));

            switch (currentState) {//run the state of the currentState index
                case 0:
                    //drives in an arc towards the first beacon
                    state.arcTowardsBeacon.run();
                    break;
                case 1:
                    //strafes right until the ODS sensor detects the white line
                    state.scanForLine.run();
                    break;
                case 2:
                    //drives straight forward until the robot is touching the beacon
                    state.driveTowardsBeacon.run();
                    break;
                case 3:
                    //detect color and activate button pusher servo
                    state.pushBeaconButton.run();
                    break;
                case 4:
                    //back away from beacon to prepare for launching ball
                    state.backAwayFromBeacon.run();
                    break;
                case 5:
                    //wait for robot to stop moving
                    sleep(500);
                    currentState++;
                    break;
                case 6:
                    //shoot ball and activate vex motors
                    state.shootball.run();
                    break;
                case 7:
                    state.sleep1000.run();
                    break;
                case 8:
                    //shoot ball and stop vex motors
                    state.shootball2.run();
                    break;
                case 9:
                    //drive forward to get within range of the taped lines
                    state.correctStrafe.run();
                    break;
                case 10:
                    //strafe right close to second beacon
                    state.slideToTheRight.run();
                    break;
                case 11:
                    //strafe right until the ODS sensor detects the tape line
                    state.scanForLine.run();
                    break;
                case 12:
                    //drives straight forward until the robot is touching the beacon
                    state.driveTowardsBeacon.run();
                    break;
                case 13:
                    //detect color and activate button pusher servo
                    state.pushBeaconButton.run();
                    break;
                case 14:
                    //pivot robot backwards towards center vortex
                    state.pivotbeacon.run();
                    break;
                case 15:
                    //backs up the distance to get to the center vortex
                    state.backuptovortex.run();
                    break;
                default:
                    robot.move(0, 0, 0);
                    robotconfig.addlog(dl, "StateMachine", "stop requested");
                    requestOpModeStop();
                    break;
            }

            //robotconfig.addlog(dl, "Mainline", "Ending state machine pass");

        }

        robot.setMotorPower(0);
        robot.pushButton(0);

        robotconfig.addlog(dl, "autonomous", "Done with opmode, exited based on OpmodeIsActive false");

    }

}
