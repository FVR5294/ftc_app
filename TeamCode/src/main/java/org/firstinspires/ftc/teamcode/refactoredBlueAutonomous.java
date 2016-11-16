package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import static org.firstinspires.ftc.teamcode.robotconfig.dl;

/**
 * Created by mail2 on 11/15/2016.
 */

@Autonomous(name = "blue autonomous", group = "refactored")

public class refactoredBlueAutonomous extends LinearOpMode {

    public static int currentBlueState = 0;
    public static robotconfig blueRobot = new robotconfig();
    public preciseMovement p = new preciseMovement();
    public stateslist state = new stateslist(this, -1);

    @Override
    public void runOpMode() {
        blueRobot.init(this);  // send whole LinearOpMode object and context
        robotconfig.addlog(dl, "autonomous", "Done with robot.init --- starting p.init");
        p.init(blueRobot, this);
        robotconfig.addlog(dl, "autonomous", "Done with pm.init --- waiting for start");
        telemetry.addData("Say", "Hello Driver");
        telemetry.update();
        waitForStart();
        robotconfig.addlog(dl, "autonomous", "Started");

        while (opModeIsActive()) {

            robotconfig.addlog(dl, "Mainline", "Begining state machine pass");

            switch (currentBlueState) {
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

        blueRobot.setMotorPower(0);
        blueRobot.pushButton(0);

        robotconfig.addlog(dl, "autonomous", "Done with opmode, exited based on OpmodeIsActive false");

    }
}
