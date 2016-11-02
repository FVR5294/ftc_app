package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by mail2 on 10/31/2016.
 */
@TeleOp(name = "master autonomous program", group = "2016")

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
        telemetry.addData("Say", "Press either A or X to select a color");
        updateTelemetry(telemetry);

        int color = 0;//1=red -1=blue

        while (color == 0) {
            if (gamepad1.b || gamepad2.b)
                color = 1;
            if (gamepad1.x || gamepad2.b)
                color = -1;
            idle();
        }

        telemetry.addData("Say", "Running program number %d", color);//if this doesn't work well enough, we could turn it into 2 separate programs

        waitForStart();
        runtime.reset();
        p.move((m.tileLength - m.robotDepth) / 2f, 0f, 0f);
        p.moveInFragments(m.tileLength * 2f, color * m.tileLength * -2f, color * -90f, 4);//if this works the way it is supposed to, it will look awesome

        robot.pushButton(robot.detectColor() * color);//see why this is a switch, Ben?
        //detect color returns 1 for red
        //color is multiplied by -1 if it is trying to get blue
        //push button accepts the 1 for red and pushes the right (pun intended) button

        p.move(m.mmPerInch * -4f, 0f, 0f); //back away from the button
        robot.pushButton(0);//reset button pusher to prevent accidental press at next beacon
        p.move(0f, color * m.tileLength * 2f, 0f);//slide to the right/left
        p.move(m.mmPerInch * 4.01f, 0f, 0f); //get next button

        robot.pushButton(robot.detectColor() * color);//admit it, my way is better than your's

        p.move(-(m.tileLength - m.robotDepth) / 2f, 0f, 0f);

        p.move(-m.tileLength * 2f, -m.tileLength * color * 2f, 0f);

        idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
    }
}
