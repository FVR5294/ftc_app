package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import static org.firstinspires.ftc.teamcode.robotconfig.dl;

/**
 * Created by mail2 on 10/31/2016.
 */
@Autonomous(name = "master autonomous program?", group = "2016")
@Disabled
public class autonomous extends LinearOpMode {
    public static int color = 1;
    public robotconfig robot = new robotconfig();
    public preciseMovement p = new preciseMovement();

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

        double speed = 0.72;

        robot.setMotorPower(speed);

        p.move(11 * measurements.mmPerInch, 0, 0, 1, robot, telemetry);

        p.automaticSquareUp(robot, telemetry);

        p.move(0, 0, -color * 45, 1, robot, telemetry);

        p.automaticSquareUp(robot, telemetry);

        p.move(52 * measurements.mmPerInch, 0, 0, 5, robot, telemetry);

        p.move(0, 0, -color * 45, 1, robot, telemetry);

        p.automaticSquareUp(robot, telemetry);

        p.move(5.5 * measurements.mmPerInch, 0, 0, 1, robot, telemetry);


        while (!robot.detectLine() && opModeIsActive()) {
            p.move(0, color * 10, 0, 1, robot, telemetry);
        }

        p.automaticSquareUp(robot, telemetry);

        while (!robot.touchBeacon.isPressed()) {
            p.move(10, 0, 0, 1, robot, telemetry);
        }

        robot.pushButton(robot.detectColor() * color);
        sleep(500);

        p.move(-2 * measurements.mmPerInch, 0, 0, 1, robot, telemetry);

        p.automaticSquareUp(robot, telemetry);

        robot.pushButton(0);

        p.move(0, 55 * measurements.mmPerInch * color, 0, 5, robot, telemetry);

        p.automaticSquareUp(robot, telemetry);

        while (!robot.detectLine() && opModeIsActive()) {
            p.move(0, color * 10, 0, 1, robot, telemetry);
        }

        p.automaticSquareUp(robot, telemetry);

        while (!robot.touchBeacon.isPressed()) {
            p.move(10, 0, 0, 1, robot, telemetry);
        }

        robot.pushButton(robot.detectColor() * color);

        sleep(500);

        p.automaticSquareUp(robot, telemetry);

        p.move(-4 * measurements.mmPerInch, 0, 0, 1, robot, telemetry);

        p.automaticSquareUp(robot, telemetry);

        robot.setMotorPower(1);

        p.move(-64 * measurements.mmPerInch, -64 * measurements.mmPerInch * color, 0, 5, robot, telemetry);

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
