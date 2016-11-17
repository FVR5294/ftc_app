package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by mail2 on 11/4/2016.
 */

@Autonomous(name = "box drills", group = "2016")
@Disabled
public class driveToWall extends LinearOpMode {
    public robotconfig robot = new robotconfig();
    public preciseMovement p = new preciseMovement();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(this);
        p.init(robot, this);
        idle();
        waitForStart();
        while (opModeIsActive()) {
            p.move(0, 0, 90, 5, robot, telemetry);
            p.move(4 * measurements.tileLength, 0, 0, 5, robot, telemetry);
            sleep(1000);
            p.automaticSquareUp(robot, telemetry);
            sleep(1000);
        }
    }
}
