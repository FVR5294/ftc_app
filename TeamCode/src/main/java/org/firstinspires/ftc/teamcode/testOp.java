/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name = "testOp", group = "2016")

public class testOp extends OpMode {

    /* Declare OpMode members. */
    robotconfig robot = new robotconfig();

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(this);
        robot.move(0, 0, 0);
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");
        updateTelemetry(telemetry);
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        double forward;
        double right;
        double spin;

        //get values from joystick
        forward = -scale(gamepad1.left_stick_y);
        right = scale(gamepad1.left_stick_x);
        spin = scale(gamepad1.right_stick_x);

        robot.move(forward, right, spin);

        if (gamepad1.x || gamepad2.x) {
            robot.pushButton(-1);
        }


        if (gamepad1.b || gamepad2.b) {
            robot.pushButton(1);
        }

        //reset servo if neither button is pressed
        if (!gamepad1.x && !gamepad2.x && !gamepad1.a && !gamepad2.a && !gamepad1.b && !gamepad2.b && !gamepad1.y && !gamepad2.y) {
            robot.pushButton(0);
        }

        if (gamepad1.a || gamepad2.a) {
            robot.pushButton(robot.detectColor());
        }

        if (gamepad1.y || gamepad2.y) {
            robot.pushButton(robot.detectColor() * -1);
        }

        if (gamepad1.left_bumper || gamepad2.left_bumper) {
            robot.spinner.setPower(1);
        }

        if (gamepad1.right_bumper || gamepad2.right_bumper) {
            robot.spinner.setPower(-1);
        }

        if (gamepad1.right_trigger > 0.1 || gamepad2.right_trigger > 0.1 || gamepad1.left_trigger > 0.1 || gamepad2.left_trigger > 0.1) {
            robot.spinner.setPower(0);
        }

        // Send telemetry message to signify robot running;
        telemetry.addData("forward", "%.2f", forward);
        telemetry.addData("right", "%.2f", right);
        telemetry.addData("spin", "%.2f", spin);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        robot.move(0, 0, 0);
    }

    public double scale(double input) {//returns an interesting set of arcs
        if (input > 0)
            return (1 - Math.sqrt(1 - input * input));
        else
            return (Math.sqrt(1 - input * input) - 1);
    }

}
