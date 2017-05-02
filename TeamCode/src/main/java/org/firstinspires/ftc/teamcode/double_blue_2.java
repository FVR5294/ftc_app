package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import static org.firstinspires.ftc.teamcode.robotconfig.dl;
import static org.firstinspires.ftc.teamcode.stateslist.currentState;
import static org.firstinspires.ftc.teamcode.stateslist.robot;

/**
 * Autonomous for blue side that shoots 2 balls
 */
//@Autonomous(name = "double blue 2", group = "blue")

public class double_blue_2 extends LinearOpMode {
    private preciseMovement p = new preciseMovement();
    private stateslist state = new stateslist();

    @Override
    public void runOpMode() {
        robot.init(this);  // send whole LinearOpMode object and context
        robotconfig.addlog(dl, "autonomous", "Done with robot.init --- waiting for start in " + this.getClass().getSimpleName());
        state.color = -1;//tell the state list what the current color is
        currentState = 0;//run each state multiple times until the state increases the currentState variable by 1
        telemetry.addData("Say", "Hello Driver - debug mode is " + robotconfig.debugMode);
        telemetry.update();
        waitForStart();
        robotconfig.addlog(dl, "autonomous", "Started");

        while (opModeIsActive()) {

            //robotconfig.addlog(dl, "Mainline", "Beginning state machine pass " + String.format(Locale.ENGLISH, "%d", currentState));

            switch (currentState) {//run the state of the currentState index
                case 0:
                    state.arcTowardsBeacon.run();
                    break;
                case 1:
                    state.scanForLine.run();
                    break;
                case 2:
                    state.driveTowardsBeacon.run();
                    break;
                case 3:
                    state.pushBeaconButton.run();
                    break;
                case 4:
                    state.backAwayFromBeacon.run();
                    break;
                case 5:
                    state.sleep500.run();
                    break;
                case 6:
                    state.shootball.run();
                    break;
                case 7:
                    state.sleep2000.run();
                    break;
                case 8:
                    state.shootball2.run();
                    break;
                case 9:
                    state.correctStrafe.run();
                    break;
                case 10:
                    state.slideToTheRight2.run();
                    break;
                case 11:
                    state.scanForLine.run();
                    break;
                case 12:
                    state.driveTowardsBeacon.run();
                    break;
                case 13:
                    state.pushBeaconButton.run();
                    break;
                case 14:
                    //pivot robot backwards towards center vortex
                    state.pivotbeaconmore.run();
                    break;
                case 15:
                    //backs up the distance to get to the center vortex
                    state.rotate60.run();
                    break;
                case 16:
                    //backs up the distance to get to the center vortex
                    state.backup84.run();
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
