package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by mail2 on 10/31/2016.
 */
@Autonomous(name = "master autonomous program", group = "2016")

/***
 * for this file, position robot flat against wall facing center vortex
 */
public class autonomous extends LinearOpMode {

    public robotconfig robot = new robotconfig();
    public preciseMovement p = new preciseMovement();
    public measurements m = new measurements();

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);
        p.init(robot);
        idle();
        int color = 1;//1=red -1=blue

        telemetry.addData("Say", "Running program number %d", color);

        waitForStart();
        runtime.reset();
        p.move((m.tileLength - m.robotDepth) / 2, 0, 0);
        waitForMotors();
        p.move(m.tileLength * 2, color * m.tileLength * -2, color * -90);//if this works the way it is supposed to, it will look awesome
        waitForMotors();
        robot.pushButton(robot.detectColor() * color);//see why this is a switch, Ben?
        //detect color returns 1 for red
        //color is multiplied by -1 if it is trying to get blue
        //push button accepts the 1 for red and pushes the right (pun intended) button

        p.move(m.mmPerInch * -4, 0, 0); //back away from the button
        waitForMotors();
        robot.pushButton(0);//reset button pusher to prevent accidental press at next beacon
        p.move(0, color * m.tileLength * 2, 0);//slide to the right/left
        waitForMotors();
        p.move(m.mmPerInch * 4.01, 0, 0); //get next button
        waitForMotors();

        robot.pushButton(robot.detectColor() * color);//admit it, my way is better than your's

        p.move(-(m.tileLength - m.robotDepth) / 2, 0, 0);
        waitForMotors();

        p.move(-m.tileLength * 2, -m.tileLength * color * 2, 0);
        waitForMotors();

    }

    public void waitForMotors() {
        double timeout = 5;
        runtime.reset();
        telemetry.addData("Path0", "Start %7d :%7d :%7d :%7d", robot.fLeftMotor.getCurrentPosition(), robot.fRightMotor.getCurrentPosition(), robot.bLeftMotor.getCurrentPosition(), robot.bRightMotor.getCurrentPosition());
        telemetry.addData("Path1", "Now   %7d :%7d :%7d :%7d", robot.fLeftMotor.getCurrentPosition(), robot.fRightMotor.getCurrentPosition(), robot.bLeftMotor.getCurrentPosition(), robot.bRightMotor.getCurrentPosition());
        telemetry.addData("Path2", "End   %7d :%7d :%7d :%7d", robot.fLeftMotor.getTargetPosition(), robot.fRightMotor.getTargetPosition(), robot.bLeftMotor.getTargetPosition(), robot.bRightMotor.getTargetPosition());
        telemetry.update();
        while (robot.isMotorBusy() && opModeIsActive() && (runtime.seconds() < timeout)) {
            telemetry.addData("Path1", "Currently at %7d :%7d :%7d :%7d", robot.fLeftMotor.getCurrentPosition(), robot.fRightMotor.getCurrentPosition(), robot.bLeftMotor.getCurrentPosition(), robot.bRightMotor.getCurrentPosition());
            telemetry.update();
            idle();
        }
    }
}
