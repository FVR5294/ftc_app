package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by mail2 on 10/31/2016.
 */
@Autonomous(name = "master autonomous program?", group = "2016")

/***
 * for this file, position robot flat against wall facing center vortex
 */
public class autonomous extends LinearOpMode {
    public robotconfig robot = new robotconfig();
    public preciseMovement p = new preciseMovement();
    public measurements m = new measurements();
    public int color = 1;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        p.init(robot, telemetry, runtime, new status());
        idle();
        waitForStart();
        telemetry.addData("Say", "Running program number %d", color);
        runtime.reset();
        p.move((m.tileLength - m.robotDepth) / 2, 0, 0);
        p.move(m.tileLength * 2, color * m.tileLength * -2, color * -90);//if this works the way it is supposed to, it will look awesome
        robot.pushButton(robot.detectColor() * color);//see why this is a switch, Ben?
        //detect color returns 1 for red
        //color is multiplied by -1 if it is trying to get blue
        //push button accepts the 1 for red and pushes the right (pun intended) button

        p.move(m.mmPerInch * -4, 0, 0); //back away from the button
        robot.pushButton(0);//reset button pusher to prevent accidental press at next beacon
        p.move(0, color * m.tileLength * 2, 0);//slide to the right/left
        p.move(m.mmPerInch * 4.01, 0, 0); //get next button

        robot.pushButton(robot.detectColor() * color);//admit it, my way is better than your's

        p.move(-(m.tileLength - m.robotDepth) / 2, 0, 0);

        p.move(-m.tileLength * 2, -m.tileLength * color * 2, 0);

    }

    /***
     * makes the function opModeIsActive() public
     */
    public class status {
        boolean call() {
            return opModeIsActive();
        }
    }
}
