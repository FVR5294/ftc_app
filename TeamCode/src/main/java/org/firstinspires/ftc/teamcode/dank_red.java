package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import static org.firstinspires.ftc.teamcode.robotconfig.dl;
import static org.firstinspires.ftc.teamcode.stateslist.currentState;
import static org.firstinspires.ftc.teamcode.stateslist.robot;

/**
 * Created by mail2 on 12/1/2016.
 * Project: ftc_app_for_2016_robot
 */

@Autonomous(name = "dank red", group = "red")

public class dank_red extends LinearOpMode {
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

            //robotconfig.addlog(dl, "Mainline", "Beginning state machine pass " + String.format(Locale.ENGLISH, "%d", currentState));

            switch (currentState) {
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
                    state.shootball.run();
                    break;
                case 6:
                    state.noscope.run();
                    break;
                case 7:
                    state.shootball.run();
                    break;
                case 8:
                    state.slideToTheRight.run();
                    break;
                case 9:
                    state.correctStrafe.run();
                    break;
                case 10:
                    state.scanForLine.run();
                    break;
                case 11:
                    state.driveTowardsBeacon.run();
                    break;
                case 12:
                    state.pushBeaconButton.run();
                    break;
                case 13:
                    state.backAwayFromBeacon.run();
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
