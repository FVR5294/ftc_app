/*
 * MIT License
 *
 * Copyright (c) 2016 Chris D
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by Chris D on 10/5/2016
 * <p>
 * In this example, you need to create a device configuration that lists two
 * "I2C Device"s, one named "mux" and the other named "ada". There are two
 * Adafruit color sensors plugged into the I2C multiplexer on ports 2 and 5.
 */
@TeleOp(name = "MultiplexColorSensorTest", group = "Iterative Opmode")
//@Disabled
public class MultiplexColorSensorTest extends OpMode {

    double vexes = 0.5;
    robotconfig robot = new robotconfig();

    @Override
    public void init() {
//        robot.init(this);
//        int milliSeconds = 48;
//        muxColor = new MultiplexColorSensor(hardwareMap, "mux", "ada",
//                ports, milliSeconds,
//                MultiplexColorSensor.GAIN_16X);
        robot.init(this);
    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {

//        muxColor.startPolling();
    }

    @Override
    public void loop() {

        vexes = -gamepad2.left_stick_y * 0.5 + 0.5 + gamepad1.right_trigger / 2 + gamepad2.right_trigger / 2 - gamepad2.left_trigger / 2;
        robot.rvex.setPosition(vexes);
        robot.lvex.setPosition(vexes);

        telemetry.addData("beacon CRGB", "%d %d %d %d",
                robot.ada.alpha(), robot.ada.red(), robot.ada.green(), robot.ada.blue());
        telemetry.addData("ODS", "%f", robot.ods.getLightDetected());
        if (robot.eject)
            telemetry.addData("intake CRGB", "%d %d %d %d",
                    robot.intake.alpha(), robot.intake.red(), robot.intake.green(), robot.intake.blue());
    }

    @Override
    public void stop() {

    }
}
