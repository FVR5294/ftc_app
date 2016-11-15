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
                    state.driveTorwardsBeacon.run();
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
                    state.driveTorwardsBeacon.run();
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
                    state.stop.run();
                    break;
                default:
                    robotconfig.addlog(dl, "error", "state not defined");
                    break;
            }
        }
    }
}
