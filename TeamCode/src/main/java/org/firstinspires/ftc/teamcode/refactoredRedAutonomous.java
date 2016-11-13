package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import static org.firstinspires.ftc.teamcode.robotconfig.dl;

/**
 * Created by mail2 on 11/13/2016.
 */

@TeleOp(name = "red autonomous", group = "refactored")

public class refactoredRedAutonomous extends LinearOpMode {
    public static final String[] states = {"clear el wall", "arc twards beacon", "siezure twards line", "stab beacon", "select color", "back away from the beacon", "drive twards next beacon"};
    public static int color = 1;
    public static int state = 0;
    public robotconfig robot = new robotconfig();
    public preciseMovement p = new preciseMovement();
    public boolean firstTime = true;
    public boolean conditionsMet = false;//if you really want to, make this list proper englishes

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
            switch (state) {
                case -1://template case for structure... something is very wrong if this is ever actually run

                    if (firstTime) {
                        robotconfig.addlog(dl, states[0], "started");
                        //set power stuff and init
                    }

                    if ("something" == "true" && "a few other things" == "true") {//checks for stuff
                        conditionsMet = true;
                    }

                    if (!conditionsMet) {
                        //run code
                    } else {
                        state++;
                        conditionsMet = false;
                        firstTime = false;
                        robotconfig.addlog(dl, states[0], "ended");
                    }
                    break;
                default:
                    robotconfig.addlog(dl, "error", "state not defined");
                    break;
            }
        }
    }
}
