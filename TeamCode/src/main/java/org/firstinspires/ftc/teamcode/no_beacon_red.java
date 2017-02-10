package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import static org.firstinspires.ftc.teamcode.robotconfig.dl;
import static org.firstinspires.ftc.teamcode.stateslist.currentState;
import static org.firstinspires.ftc.teamcode.stateslist.robot;

/**
 * Autonomous for red side that shoots 2 balls without getting beacons
 */

@Autonomous(name = "no beacon red", group = "red")

public class no_beacon_red extends LinearOpMode {
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
                case 1:
                    state.backup1.run();
                    break;
                case 0:
                    sleep(12000);
                    currentState++;
                    break;
                case 2:
                    state.shootball.run();
                    break;
                case 3:
                    sleep(2000);
                    currentState++;
                    break;
                case 4:
                    state.shootball2.run();
                    break;
                case 5:
                    state.rotateN.run();
                    break;
                case 6:
                    state.backup2.run();
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
