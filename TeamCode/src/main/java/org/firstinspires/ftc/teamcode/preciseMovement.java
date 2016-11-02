package org.firstinspires.ftc.teamcode;


import static java.lang.Thread.sleep;

/**
 * Created by mail2 on 10/31/2016.
 */

public class preciseMovement {
    private robotconfig robot;
    private measurements m = new measurements();

    public void init(robotconfig robot) {
        this.robot = robot;
        this.robot.enableMotorBreak();
        this.robot.resetMotorEncoders();
        this.robot.enableEncodersToPosition();
        this.robot.setMotorPower(1);
    }

    /***
     * uses math to convert an amount of degrees into how much distance the wheel spins
     *
     * @param degrees amount of degrees to spin clockwise, negative if backwards
     * @return distance a wheel will spin in mm
     */
    private float spin2mm(float degrees) {
        return (degrees / 360) * (m.wheelDiagonal * m.pi);
    }

    /***
     * uses math to convert a distance in millimeters to the number of pulses the motor should generate to go that far
     *
     * @param mm a distance in millimeters
     * @return number of pulses generated
     */
    private int mm2pulses(float mm) {
        return (int) ((mm / (m.pi * m.wheelDiameter)) * m.ppr);
    }

    /***
     * sets the drive train motor encoder targets to values to go a specific distance
     *
     * @param forward mm forward to move
     * @param right   mm to slide to the right
     * @param spin    degrees to spin clockwise (or negative for counter clockwise)
     */
    public void move(float forward, float right, float spin) {
        this.robot.resetMotorEncoders();
        this.robot.enableEncodersToPosition();
        this.robot.setMotorTargets(mm2pulses(forward), mm2pulses(right), mm2pulses(spin2mm(spin)));
        while (this.robot.isMotorBusy()) {
            try {
                sleep(100);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }

    /***
     * same as move but is in fragments because I think it might make a better curve
     *
     * @param forward mm forward to move
     * @param right   mm to slide to the right
     * @param spin    degrees to spin clockwise
     * @param number  number of checkpoints as a positive integer
     */
    public void moveInFragments(float forward, float right, float spin, int number) {
        if (number < 1)
            number = 1;
        for (int i = 0; i < number; i++) {
            this.move(forward / number, right / number, spin / number);
        }
    }

}
