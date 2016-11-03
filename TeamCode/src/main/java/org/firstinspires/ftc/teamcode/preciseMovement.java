package org.firstinspires.ftc.teamcode;


/**
 * Created by mail2 on 10/31/2016.
 */

public class preciseMovement {
    private robotconfig robot;
    private measurements m = new measurements();

    public void init(robotconfig robot) {
        this.robot = robot;
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


}
