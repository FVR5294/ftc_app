package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by mail2 on 11/7/2016.
 */

@TeleOp(name = "antiSpin", group = "above")

public class antiSpin extends LinearOpMode {
    public robotconfig robot = new robotconfig();
    public preciseMovement p = new preciseMovement();
    public pidController pid = new pidController(0, 0, 1);
    private double forward = 0;
    private double right = 0;
    private double spin = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(this);
        idle();
        waitForStart();
        while (opModeIsActive() && pid.tunning) {
            forward = -gamepad1.left_stick_y * 0.6;
            right = gamepad1.left_stick_x * 0.6;
//            spin = gamepad1.right_stick_x * 0.3;
            pid.pTune();
            robot.move(forward, right, spin - pid.getPID((robot.getCurrentAngle() + 360) % 360, telemetry));
            telemetry.update();
            idle();
        }
        while (opModeIsActive()) {
            forward = -gamepad1.left_stick_y;
            right = gamepad1.left_stick_x;
            spin = gamepad1.right_stick_x;
            robot.move(forward, right, spin - pid.getPID((robot.getCurrentAngle() + 360) % 360, telemetry));
            telemetry.update();
            idle();
        }
    }
}
