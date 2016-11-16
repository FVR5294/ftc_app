package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by mail2 on 11/7/2016.
 */

@Autonomous(name = "antiSpin", group = "2016")
@Disabled
public class antiSpin extends LinearOpMode {
    public robotconfig robot = new robotconfig();
    public preciseMovement p = new preciseMovement();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(this);
        p.init(robot, this);
        idle();
        waitForStart();
        while (opModeIsActive()) {
            p.automaticSquareUp(robot, telemetry);
            idle();
        }
    }
}
