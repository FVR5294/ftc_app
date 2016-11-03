package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by mail2 on 10/31/2016.
 */

/***
 * library to use math for precise movement, primarily for autonomous
 */
public class preciseMovement {
    private robotconfig robot;
    private measurements m = new measurements();
    private Telemetry telemetry;
    private ElapsedTime runtime;
    private autonomous.status status;

    public void init(robotconfig robot, Telemetry telemetry, ElapsedTime runtime, autonomous.status status) {
        this.robot = robot;
        this.telemetry = telemetry;
        this.runtime = runtime;
        this.status = status;
        //this.robot.enableMotorBreak();
        this.robot.resetMotorEncoders();
        Thread.yield();
        this.robot.enableEncodersToPosition();
        Thread.yield();
        this.robot.setMotorPower(1);
    }

    /***
     * uses math to convert an amount of degrees into how much distance the wheel spins
     *
     * @param degrees amount of degrees to spin clockwise, negative if backwards
     * @return distance a wheel will spin in mm
     */
    private double spin2mm(double degrees) {
        return (degrees / 360) * (m.wheelDiagonal * m.pi);
    }

    /***
     * uses math to convert a distance in millimeters to the number of pulses the motor should generate to go that far
     *
     * @param mm a distance in millimeters
     * @return number of pulses generated
     */
    private int mm2pulses(double mm) {
        return (int) ((mm / (m.pi * m.wheelDiameter)) * m.ppr);
    }

    /***
     * sets the drive train motor encoder targets to values to go a specific distance
     *
     * @param forward mm forward to move
     * @param right   mm to slide to the right
     * @param spin    degrees to spin clockwise (or negative for counter clockwise)
     */
    public void move(double forward, double right, double spin) {
        this.robot.resetMotorEncoders();
        Thread.yield();
        this.robot.enableEncodersToPosition();
        Thread.yield();
        this.robot.setMotorTargets(mm2pulses(forward), mm2pulses(right), mm2pulses(spin2mm(spin)));
    }

    public void waitForMotors() {
        double timeout = 5;
        runtime.reset();
        telemetry.addData("Path0", "Start %7d :%7d :%7d :%7d", robot.fLeftMotor.getCurrentPosition(), robot.fRightMotor.getCurrentPosition(), robot.bLeftMotor.getCurrentPosition(), robot.bRightMotor.getCurrentPosition());
        telemetry.addData("Path1", "Now   %7d :%7d :%7d :%7d", robot.fLeftMotor.getCurrentPosition(), robot.fRightMotor.getCurrentPosition(), robot.bLeftMotor.getCurrentPosition(), robot.bRightMotor.getCurrentPosition());
        telemetry.addData("Path2", "End   %7d :%7d :%7d :%7d", robot.fLeftMotor.getTargetPosition(), robot.fRightMotor.getTargetPosition(), robot.bLeftMotor.getTargetPosition(), robot.bRightMotor.getTargetPosition());
        telemetry.update();
        boolean isRunning = true;
        while (robot.isMotorBusy() && (runtime.seconds() < timeout) && isRunning) {
            telemetry.addData("Path1", "Currently at %7d :%7d :%7d :%7d", robot.fLeftMotor.getCurrentPosition(), robot.fRightMotor.getCurrentPosition(), robot.bLeftMotor.getCurrentPosition(), robot.bRightMotor.getCurrentPosition());
            telemetry.update();
            isRunning = status.call();
            Thread.yield();
        }
    }

}
