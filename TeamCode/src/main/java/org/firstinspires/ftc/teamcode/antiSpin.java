package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.stormbots.MiniPID;

/**
 * Created by mail2 on 11/7/2016.
 */

@TeleOp(name = "antiSpin", group = "above")

public class antiSpin extends LinearOpMode {
    public robotconfig robot = new robotconfig();
    public preciseMovement p = new preciseMovement();

    private double forward = 0;
    private double right = 0;
    private double spin = 0;

    private double pGain = 0;
    private double iGain = 0;
    private double dGain = 0;
    private double output = 0;

    MiniPID pid = new MiniPID(pGain, iGain, dGain);

    private boolean positiveOutput = false;
    private int timeCount = 0;
    private int ntimeCount = 0;

    private ElapsedTime loopTimer = new ElapsedTime();
    private double loopTime = 0;
    private double waitTime = 50;

    @Override
    public void runOpMode() throws InterruptedException {
        pid.setDirection(true);
        pid.setOutputLimits(-1, 1);
        robot.init(this);
        idle();
        waitForStart();
        while (opModeIsActive()) {

            if (gamepad1.a) {
                pGain -= gamepad1.left_stick_y / 100;
                pid.setP(pGain);
            } else if (gamepad1.b) {
                iGain -= gamepad1.left_stick_y / 100;
                pid.setI(iGain);
            } else if (gamepad1.y) {
                dGain -= gamepad1.left_stick_y / 100;
                pid.setD(dGain);
            } else if (gamepad1.left_bumper) {
                iGain = 0;
                dGain = 0;
                pGain = pGain / 0.60;
                pid.setP(pGain);
                pid.setI(iGain);
                pid.setD(dGain);
            } else if (gamepad1.right_bumper) {
                iGain = 1.2 * pGain / timeCount;
                dGain = 3 * pGain * timeCount / 40;
                pGain = pGain * 0.60;
                //https://en.wikipedia.org/wiki/Ziegler%E2%80%93Nichols_method
                pid.setP(pGain);
                pid.setI(iGain);
                pid.setD(dGain);
                while (gamepad1.right_bumper) {
                    idle();
                }
            } else {
                forward = -gamepad1.left_stick_y;
                right = gamepad1.left_stick_x;
            }

            spin += gamepad1.right_stick_x;

            output = pid.getOutput(robot.getCurrentAngle(), spin);
            robot.move(forward, right, output);

            ntimeCount++;

            if (positiveOutput) {

                if (output < 0) {
                    positiveOutput = false;
                }

            } else {

                if (output > 0) {
                    positiveOutput = true;
                    timeCount = ntimeCount;
                    ntimeCount = 0;
                }

            }

            telemetry.addData("spin", "%f", spin);
            telemetry.addData("output", "%f", output);
            telemetry.addLine();
            telemetry.addData("timeCount", "%d", timeCount);
            telemetry.addData("pGain", "%f", pGain);
            telemetry.addData("iGain", "%f", iGain);
            telemetry.addData("dGain", "%f", dGain);
            telemetry.addLine();
            telemetry.addData("loopTime", "%.0f", loopTime);
            telemetry.addData("waitTime", "%.0f", waitTime);
            telemetry.update();

            loopTime = loopTimer.milliseconds();
            waitTime = Math.max(0, 50 - loopTime);
            loopTimer.reset();

            sleep((long) waitTime);
        }
    }
}
