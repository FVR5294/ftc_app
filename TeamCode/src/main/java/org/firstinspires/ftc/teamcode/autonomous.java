package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by mail2 on 10/31/2016.
 */
@Autonomous(name = "master autonomous program?", group = "2016")

/***
 * for this file, position robot flat against wall facing center vortex
 */
public class autonomous extends LinearOpMode {
    public static int color = 1;
    public robotconfig robot = new robotconfig();
    public preciseMovement p = new preciseMovement();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        p.init(robot);
        idle();
        waitForStart();
        telemetry.addData("Say", "Running program number %d", color);
        p.move(Math.abs(measurements.tileLength - measurements.robotDepth) / 2, 0, 0, 3, robot, telemetry);
        p.move(measurements.tileLength, color * measurements.tileLength * -1, 0, 3, robot, telemetry);
        p.move(0, 0, color * -90, 3, robot, telemetry);
        p.move(0, color * measurements.tileLength, 0, 3, robot, telemetry);
        p.move(measurements.tileLength, 0, 0, 3, robot, telemetry);
        robot.pushButton(robot.detectColor() * color);//see why this is a switch, Ben?
        //detect color returns 1 for red
        //color is multiplied by -1 if it is trying to get blue
        //push button accepts the 1 for red and pushes the right (pun intended) button

        p.move(measurements.mmPerInch * -4, 0, 0, 3, robot, telemetry); //back away from the button
        robot.pushButton(0);//reset button pusher to prevent accidental press at next beacon
        p.move(0, color * measurements.tileLength * 2, 0, 3, robot, telemetry);//slide to the right/left
        p.move(measurements.mmPerInch * 4.01, 0, 0, 3, robot, telemetry); //get next button

        robot.pushButton(robot.detectColor() * color);//admit it, my way is better than your's

        p.move(-Math.abs(measurements.tileLength - measurements.robotDepth) / 2, 0, 0, 3, robot, telemetry);

        p.move(-measurements.tileLength * 2, -measurements.tileLength * color * 2, 0, 3, robot, telemetry);

        robot.setMotorPower(0);

        while (opModeIsActive()) {
            telemetry.addData("Path", "at %7d :%7d :%7d :%7d", robot.fLeftMotor.getCurrentPosition(), robot.fRightMotor.getCurrentPosition(), robot.bLeftMotor.getCurrentPosition(), robot.bRightMotor.getCurrentPosition());
            telemetry.update();
            idle();
        }

    }
}
