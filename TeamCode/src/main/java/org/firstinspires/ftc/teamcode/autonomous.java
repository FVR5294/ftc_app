package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.teamcode.robotconfig.dl;

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
    public ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() {

        telemetry.addData("00:Opmode ", " Running program");
        telemetry.update();
        robot.init(this);  // send whole LinearOpMode object and context
        robotconfig.addlog(dl, "autonomous", "Done with robot.init --- starting p.init");
        p.init(robot, this);
        robotconfig.addlog(dl, "autonomous", "Done with pm.init --- waiting for start");

        waitForStart();
        robotconfig.addlog(dl, "autonomous", "Started");

        double speed = 0.65;

        robot.setMotorPower(speed);

        p.move(8 * measurements.mmPerInch, 0, 0, 2, robot, telemetry);

        p.automaticSquareUp(robot, telemetry);

        p.move(0, 0, -color * 45, 3, robot, telemetry);

        p.automaticSquareUp(robot, telemetry);

        p.move(55 * measurements.mmPerInch, 0, 0, 3, robot, telemetry);

        p.automaticSquareUp(robot, telemetry);

        p.move(0, 0, -color * 45, 2, robot, telemetry);

        p.automaticSquareUp(robot, telemetry);

        timer.reset();
        while (!robot.detectLine() && timer.seconds() < 5) {
            p.move(0, color * 0.25, 0, 1, robot, telemetry);
        }

        p.automaticSquareUp(robot, telemetry);

        p.move(10 * measurements.mmPerInch, 0, 0, 3, robot, telemetry);

        robot.setMotorPower(0);
        robot.pushButton(robot.detectColor() * color);
        robot.setMotorPower(speed);

        p.move(-2 * measurements.mmPerInch, 0, 0, 1, robot, telemetry);

        p.automaticSquareUp(robot, telemetry);

        robot.pushButton(0);

        p.move(0, 50 * measurements.mmPerInch * color, 0, 3, robot, telemetry);

        p.automaticSquareUp(robot, telemetry);

        while (!robot.detectLine() && timer.seconds() < 5) {
            p.move(0, color * 0.5, 0, 1, robot, telemetry);
        }

        p.automaticSquareUp(robot, telemetry);

        p.move(2 * measurements.mmPerInch, 0, 0, 1, robot, telemetry);

        robot.setMotorPower(0);
        robot.pushButton(robot.detectColor() * color);
        robot.setMotorPower(speed);

        p.move(-4 * measurements.mmPerInch, 0, 0, 1, robot, telemetry);

        p.automaticSquareUp(robot, telemetry);

        p.move(-64 * measurements.mmPerInch, -64 * measurements.mmPerInch * color, 0, 3, robot, telemetry);

        p.automaticSquareUp(robot, telemetry);

        p.move(-4 * measurements.mmPerInch, 0, 0, 1, robot, telemetry);

        robot.setMotorPower(0);
        robotconfig.addlog(dl, "autonomous", "Autonomous is complete");
        while (opModeIsActive()) {
            telemetry.addData("Path", "at %7d :%7d :%7d :%7d", robot.fLeftMotor.getCurrentPosition(), robot.fRightMotor.getCurrentPosition(), robot.bLeftMotor.getCurrentPosition(), robot.bRightMotor.getCurrentPosition());
            telemetry.update();
            idle();
        }

    }
}
