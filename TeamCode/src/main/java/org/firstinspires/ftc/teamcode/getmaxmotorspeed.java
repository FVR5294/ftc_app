package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by mail2 on 12/9/2016.
 * Project: ftc_app_for_2016_robot
 */

@TeleOp(name = "get max motor speed")

public class getmaxmotorspeed extends OpMode {

    robotconfig robot = new robotconfig();
    ElapsedTime timer = new ElapsedTime();
    int encodercount = 0;

    @Override
    public void init() {
        robot.init(this);
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");
        updateTelemetry(telemetry);
    }

    @Override
    public void init_loop() {
        timer.reset();
    }

    @Override
    public void loop() {
        timer.reset();
    }
}
