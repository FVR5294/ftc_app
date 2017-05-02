package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import static org.firstinspires.ftc.teamcode.robotconfig.dl;
import static org.firstinspires.ftc.teamcode.stateslist.currentState;
import static org.firstinspires.ftc.teamcode.stateslist.robot;

/**
 * Created by mail2 on 12/1/2016.
 * Project: ftc_app_for_2016_robot
 */

@Autonomous(name = "dank blue", group = "blue")
@Disabled
public class dank_blue extends LinearOpMode {
    private preciseMovement p = new preciseMovement();
    private stateslist state = new stateslist();

    @Override
    public void runOpMode() {
        robot.init(this);  // send whole LinearOpMode object and context
        robotconfig.addlog(dl, "autonomous", "Done with robot.init --- waiting for start in " + this.getClass().getSimpleName());
        state.color = -1;
        currentState = 0;
        telemetry.addData("Say", "Hello Driver - debug mode is " + robotconfig.debugMode);
        telemetry.update();
        waitForStart();
        robotconfig.addlog(dl, "autonomous", "Started");

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
                sleep(500);
                currentState++;
                break;
            case 6:
                state.shootball.run();
                break;
            case 7:
                state.noscope.run();
                break;
            case 8:
                state.shootball2.run();
                break;
            case 9:
                state.correctStrafe.run();
                break;
            case 10:
                state.slideToTheRight.run();
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
                state.pivotbeacon.run();
                break;
            case 15:
                state.backuptovortex.run();
                break;
            default:
                robot.move(0, 0, 0);
                robotconfig.addlog(dl, "StateMachine", "stop requested");
                requestOpModeStop();
                break;
        }

        robot.setMotorPower(0);
        robot.pushButton(0);

        robotconfig.addlog(dl, "autonomous", "Done with opmode, exited based on OpmodeIsActive false");

    }
}
