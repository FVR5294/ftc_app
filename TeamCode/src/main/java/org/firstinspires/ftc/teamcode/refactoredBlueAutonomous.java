package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import static org.firstinspires.ftc.teamcode.robotconfig.dl;
import static org.firstinspires.ftc.teamcode.stateslist.currentState;
import static org.firstinspires.ftc.teamcode.stateslist.robot;

/**
 * Created by mail2 on 11/15/2016.
 * Project: ftc_app_for_2016_robot
 */

@Autonomous(name = "blue autonomous", group = "refactored")

public class refactoredBlueAutonomous extends LinearOpMode {

    private preciseMovement p = new preciseMovement();
    private stateslist state = new stateslist();

    @Override
    public void runOpMode() {
        robot.init(this);  // send whole LinearOpMode object and context
        robotconfig.addlog(dl, "autonomous", "Done with robot.init --- starting p.init");
        p.init(robot, this);
        robotconfig.addlog(dl, "autonomous", "Done with pm.init --- waiting for start");
        state.color = -1;
        telemetry.addData("Say", "Hello Driver");
        telemetry.update();
        waitForStart();
        robotconfig.addlog(dl, "autonomous", "Started");

        while (opModeIsActive()) {

            robotconfig.addlog(dl, "Mainline", "Begining state machine pass");

            switch (currentState) {
                case 0:
                    state.clearWall.run();
                    break;
                case 1:
                    state.arcTorwardsBeacon.run();
                    break;
                case 2:
                    state.getCloserToWall.run();
                    break;
                case 3:
                    state.scanForLine.run();
                    break;
                case 4:
                    state.driveTowardsBeacon.run();
                    break;
                case 5:
                    state.pushBeaconButton.run();
                    break;
                case 6:
                    state.backAwayFromBeacon.run();
                    break;
                case 7:
                    state.driveToNextBeacon.run();
                    break;
                case 8:
                    state.scanForLine.run();
                    break;
                case 9:
                    state.driveTowardsBeacon.run();
                    break;
                case 10:
                    state.pushBeaconButton.run();
                    break;
                case 11:
                    state.backAwayFromBeacon.run();
                    break;
                case 12:
                    state.retreatToCenter.run();
                    break;
                case 13:
                    state.driveOnToWood.run();
                    break;
                case 14:
                    requestOpModeStop();
                    robotconfig.addlog(dl, "StateMachine", "stop requested");
                    break;
                default:
                    robotconfig.addlog(dl, "error", "state not defined");
                    break;
            }

            robotconfig.addlog(dl, "Mainline", "Ending state machine pass");

        }

        robot.setMotorPower(0);
        robot.pushButton(0);

        robotconfig.addlog(dl, "autonomous", "Done with opmode, exited based on OpmodeIsActive false");

    }
}
