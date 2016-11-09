package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import static org.firstinspires.ftc.teamcode.robotconfig.dl;

/**
 * Created by mail2 on 10/31/2016.
 */

/***
 * library to use math for precise movement, primarily for autonomous
 */
public class preciseMovement {
    public robotconfig robot;
    public LinearOpMode linearOpMode;
    public Telemetry ltelemetry;
    private measurements m = new measurements();
    private ElapsedTime runtime = new ElapsedTime();

    public void init(robotconfig robot, LinearOpMode linearOpMode) {
        //this.robot.enableMotorBreak();

        this.linearOpMode = linearOpMode;
        this.robot = robot;

        // Save reference to Hardware map in class variable

        robotconfig.addlog(dl, "pm.init", "pm.init was invoked");

        robot.resetMotorEncoders();
        Thread.yield();
        robot.setMotorTargets(0, 0, 0);
        robot.enableEncodersToPosition();
        Thread.yield();
        robot.setMotorPower(1);

        robotconfig.addlog(dl, "pm.init", "pm.init finished");
    }

    /***
     * uses math to convert an amount of degrees into how much distance the wheel spins
     *
     * @param degrees amount of degrees to spin clockwise, negative if backwards
     * @return distance a wheel will spin in mm
     */
    private double spin2mm(double degrees) {
        return (degrees / 360) * (measurements.wheelDiagonal * measurements.pi);
    }

    /***
     * uses math to convert a distance in millimeters to the number of pulses the motor should generate to go that far
     *
     * @param mm a distance in millimeters
     * @return number of pulses generated
     */
    private int mm2pulses(double mm) {
        return (int) ((measurements.ppr / (measurements.pi * measurements.wheelDiameter)) * mm);
    }

    /***
     * function takes measurements in mm to move the robot
     *
     * @param forward amount to move forward in mm
     * @param right amount to move right in mm
     * @param spin amount to spin degrees clockwise
     * @param timeout time to take doing movement
     * @param robot just make it robot, for getting the configuration
     * @param telemetry just leave it as telemetry to allow program to write telemetry
     */
    public void move(double forward, double right, double spin, double timeout, robotconfig robot, Telemetry telemetry) {
        telemetry.addData("colorSensor", "Green: %d", robot.muxColor.getCRGB(robot.ports[1])[2]);
        robotconfig.addlog(dl, "pm.move", "called with right:" + String.format("%.2f", right) + " and spin:" + String.format("%.2f", spin));
        robot.setMotorTargets(mm2pulses(forward), mm2pulses(right), mm2pulses(spin2mm(spin)));
        waitForMotors(robot, telemetry, forward, right, spin, timeout);
    }

    public void waitForMotors(robotconfig robot, Telemetry telemetry, double forward, double right, double spin, double timeout) {
        runtime.reset();
        while (robot.isMotorBusy() && (runtime.seconds() < timeout) && linearOpMode.opModeIsActive()) {
            telemetry.addData("Path0", "Go to %f in : %f in : %f in", (forward / measurements.mmPerInch), (right / measurements.mmPerInch), spin);
            robotconfig.addlog(dl, "pm.waitforMotors", "called with right:" + String.format("%.2f", right) + " and spin:" + String.format("%.2f", spin) + " and forward:" + String.format("%.2f", forward));
            telemetry.addData("Path0", "Go to %f in : %f in : %f in", (forward / measurements.mmPerInch), (right / measurements.mmPerInch), spin);
            telemetry.addData("Path1", "At  %7d :%7d :%7d :%7d", robot.fLeftMotor.getCurrentPosition(), robot.fRightMotor.getCurrentPosition(), robot.bLeftMotor.getCurrentPosition(), robot.bRightMotor.getCurrentPosition());
            telemetry.addData("Path2", "End %7d :%7d :%7d :%7d", robot.fLeftMotor.getTargetPosition(), robot.fRightMotor.getTargetPosition(), robot.bLeftMotor.getTargetPosition(), robot.bRightMotor.getTargetPosition());
            telemetry.update();
            Thread.yield();
        }
        // Need to gracefully exit loop here as we have either timed out or a stop has been requested

        if (runtime.seconds() >= timeout) {
            robotconfig.addlog(dl, "pm.waitforMotors", "timed out: " + String.format("%.2f", runtime.seconds()));
        } else if (!linearOpMode.opModeIsActive()) {
            robotconfig.addlog(dl, "pm.waitforMotors", "Stop of opmode was requested");
        } else {
            robotconfig.addlog(dl, "pm.waitforMotors", "exited move normally");
        }
    }

    /***
     * is supposed to square up the robot to the nearest 45 degrees
     *
     * @param robot     should be robot
     * @param telemetry should be telemetry
     */
    public void automaticSquareUp(robotconfig robot, Telemetry telemetry) {
        this.move(0, 0, (robot.getCurrentAngle() - (Math.round(robot.getCurrentAngle() / 45) * 45)), 1, robot, telemetry);
    }

}
