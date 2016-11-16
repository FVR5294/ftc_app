package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import static org.firstinspires.ftc.teamcode.robotconfig.dl;

/**
 * Created by mail2 on 11/13/2016.
 */

@TeleOp(name = "red autonomous", group = "refactored")

public class refactoredRedAutonomous extends LinearOpMode {

    public static int color = 1;
    public static int currentState = 0;
    public robotconfig robot = new robotconfig();
    public preciseMovement p = new preciseMovement();
    public stateslist state = new stateslist();

    @Override
    public void runOpMode() {
        robot.init(this);  // send whole LinearOpMode object and context
        robotconfig.addlog(dl, "autonomous", "Done with robot.init --- starting p.init");
        p.init(robot, this);
        robotconfig.addlog(dl, "autonomous", "Done with pm.init --- waiting for start");
        telemetry.addData("Say", "Hello Driver");
        telemetry.update();
        waitForStart();
        robotconfig.addlog(dl, "autonomous", "Started");

        while (opModeIsActive()) {

            robotconfig.addlog(dl, "Mainline", "Begining state machine pass");

            switch (currentState) {
                case 0:
                    robotconfig.addlog(dl, "StateMachine", "Executing clearWall state");
                    state.clearWall.run();
                    robotconfig.addlog(dl, "StateMachine", "clearWall state execution complete");
                    break;
                case 1:
                    robotconfig.addlog(dl, "StateMachine", "Executing arcTorwardsBeacon state");
                    state.arcTorwardsBeacon.run();
                    robotconfig.addlog(dl, "StateMachine", "arcTorwardsBeacon state execution complete");
                    break;
                case 2:
                    robotconfig.addlog(dl, "StateMachine", "Executing getCloserToWall state");
                    state.getCloserToWall.run();
                    robotconfig.addlog(dl, "StateMachine", "getCloserToWall state execution complete");
                    break;
                case 3:
                    robotconfig.addlog(dl, "StateMachine", "Executing scanForLine state");
                    state.scanForLine.run();
                    robotconfig.addlog(dl, "StateMachine", "scanForLine state execution complete");
                    break;
                case 4:
                    robotconfig.addlog(dl, "StateMachine", "Executing clearWall state");
                    state.driveTowardsBeacon.run();
                    robotconfig.addlog(dl, "StateMachine", "clearWall state execution complete");
                    break;
                case 5:
                    robotconfig.addlog(dl, "StateMachine", "Executing pushBeaconButton state");
                    state.pushBeaconButton.run();
                    robotconfig.addlog(dl, "StateMachine", "pushBeaconButton state execution complete");
                    break;
                case 6:
                    robotconfig.addlog(dl, "StateMachine", "Executing backAwayFromBeacon state");
                    state.backAwayFromBeacon.run();
                    robotconfig.addlog(dl, "StateMachine", "backAwayFromBeacon state execution complete");
                    break;
                case 7:
                    robotconfig.addlog(dl, "StateMachine", "Executing driveToNextBeacon state");
                    state.driveToNextBeacon.run();
                    robotconfig.addlog(dl, "StateMachine", "driveToNextBeacon state execution complete");
                    break;
                case 8:
                    robotconfig.addlog(dl, "StateMachine", "Executing scanForLine state");
                    state.scanForLine.run();
                    robotconfig.addlog(dl, "StateMachine", "scanForLine state execution complete");
                    break;
                case 9:
                    robotconfig.addlog(dl, "StateMachine", "Executing driveTowardsBeacon state");
                    state.driveTowardsBeacon.run();
                    robotconfig.addlog(dl, "StateMachine", "driveTowardsBeacon state execution complete");
                    break;
                case 10:
                    robotconfig.addlog(dl, "StateMachine", "Executing pushBeaconButton state");
                    state.pushBeaconButton.run();
                    robotconfig.addlog(dl, "StateMachine", "pushBeaconButton state execution complete");
                    break;
                case 11:
                    robotconfig.addlog(dl, "StateMachine", "Executing backAwayFromBeacon state");
                    state.backAwayFromBeacon.run();
                    robotconfig.addlog(dl, "StateMachine", "backAwayFromBeacon state execution complete");
                    break;
                case 12:
                    robotconfig.addlog(dl, "StateMachine", "Executing retreatToCenter state");
                    state.retreatToCenter.run();
                    robotconfig.addlog(dl, "StateMachine", "retreatToCenter state execution complete");
                    break;
                case 13:
                    robotconfig.addlog(dl, "StateMachine", "Executing driveOnToWood state");
                    state.driveOnToWood.run();
                    robotconfig.addlog(dl, "StateMachine", "driveOnToWood state execution complete");
                    break;
                case 14:
                    robotconfig.addlog(dl, "StateMachine", "Executing stop state");
                    state.stop.run();
                    robotconfig.addlog(dl, "StateMachine", "stop state execution complete");
                    break;
                default:
                    robotconfig.addlog(dl, "error", "state not defined");
                    break;
            }

            robotconfig.addlog(dl, "Mainline", "Ending state machine pass");

        }

        robotconfig.addlog(dl, "autonomous", "Done with opmode, exited based on OpmodeIsActive false");

    }
}
