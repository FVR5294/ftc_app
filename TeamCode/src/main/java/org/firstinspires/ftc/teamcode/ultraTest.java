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

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by Chris D on 10/5/2016
 * <p>
 * In this example, you need to create a device configuration that lists two
 * "I2C Device"s, one named "mux" and the other named "ada". There are two
 * Adafruit color sensors plugged into the I2C multiplexer on ports 2 and 5.
 */
@TeleOp(name = "Ultra Test", group = "above")
//@Disabled
public class ultraTest extends OpMode {
//    MultiplexColorSensor muxColor;
//    int[] ports = {2, 5};
//    robotconfig robot = new robotconfig();

    //    ColorSensor ada;
//    OpticalDistanceSensor ods;
    ModernRoboticsI2cRangeSensor ultra;

    @Override
    public void init() {
//        robot.init(this);
//        int milliSeconds = 48;
//        muxColor = new MultiplexColorSensor(hardwareMap, "mux", "ada",
//                ports, milliSeconds,
//                MultiplexColorSensor.GAIN_16X);
//        ada = hardwareMap.colorSensor.get("ada");
//        ods = hardwareMap.opticalDistanceSensor.get("ods");
        ultra = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "ultra");
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
//        for (int i = 0; i < ports.length; i++) {
//            int[] crgb = muxColor.getCRGB(ports[i]);

//            telemetry.addLine("Sensor " + ports[i]);
//        }

//        telemetry.addData("CRGB", "%d %d %d %d",
//                ada.alpha(), ada.red(), ada.green(), ada.blue());
//        telemetry.addData("ODS", "%f", ods.getLightDetected());
        telemetry.addData("Ultra U cm", "%f", ultra.cmUltrasonic());
        telemetry.addData("Ultra O cm", "%f", ultra.cmOptical());
        telemetry.addData("Ultra S", ultra.status());
    }

    @Override
    public void stop() {

    }
}
