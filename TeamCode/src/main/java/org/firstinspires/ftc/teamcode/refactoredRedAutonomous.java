package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.Locale;

import static org.firstinspires.ftc.teamcode.robotconfig.dl;
import static org.firstinspires.ftc.teamcode.stateslist.currentState;
import static org.firstinspires.ftc.teamcode.stateslist.robot;

/**
 * Created by mail2 on 11/13/2016.
 * Project: ftc_app_for_2016_robot
 */

@Autonomous(name = "red autonomous", group = "refactored")

public class refactoredRedAutonomous extends LinearOpMode {

    private preciseMovement p = new preciseMovement();
    private stateslist state = new stateslist();

    @Override
    public void runOpMode() {
        robot.init(this);  // send whole LinearOpMode object and context
        robotconfig.addlog(dl, "autonomous", "Done with robot.init --- waiting for start in " + this.getClass().getSimpleName());
        state.color = 1;
        currentState = 0;
        telemetry.addData("Say", "Hello Driver - debug mode is " + robotconfig.debugMode);
        telemetry.update();
        waitForStart();
        robotconfig.addlog(dl, "autonomous", "Started");

        while (opModeIsActive()) {

            robotconfig.addlog(dl, "Mainline", "Beginning state machine pass " + String.format(Locale.ENGLISH, "%d", currentState));

            switch (currentState) {
                case 0:
                    state.clearWall.run();
                    break;
                case 1:
                    state.arcTowardsBeacon.run();
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
                    state.scanForLine.run();
                    break;
                case 8:
                    state.driveTowardsBeacon.run();
                    break;
                case 9:
                    state.pushBeaconButton.run();
                    break;
                case 10:
                    state.backAwayFromBeacon.run();
                    break;
                case 11:
                    state.retreatToCenter.run();
                    break;
                case 12:
                    state.driveOnToWood.run();
                    break;
                case 13:
                    robotconfig.addlog(dl, "StateMachine", "stop requested");
                    requestOpModeStop();
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
