package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by mail2 on 11/7/2016.
 */

@TeleOp(name = "antiSpin", group = "above")

public class antiSpin extends LinearOpMode {
    public robotconfig robot = new robotconfig();
    public preciseMovement p = new preciseMovement();
    public pidController pid = new pidController(0.01, 0, 0);
    private double forward = 0;
    private double right = 0;
    private double spin = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(this);
        idle();
        waitForStart();
        while (opModeIsActive()) {
            forward = -gamepad1.left_stick_y * 0.6;
            right = gamepad1.left_stick_x * 0.6;
            spin = gamepad1.right_stick_x * 0.3;
            robot.move(forward, right, spin + pid.getPID(robot.getCurrentAngle(), telemetry));
            pid.pTune();
            telemetry.update();
            idle();
        }
    }
}
