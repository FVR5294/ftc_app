package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by mail2 on 10/27/2016.
 */

/***
 * a class designed for making a simple pid controller...
 * It could probably be used with the gyroscope sensor to reduce rotation
 */
public class pidController {
    public double sp = 0;//set target for pv

    public double pGain;
    public double iGain;
    public double dGain;

    public double p = 0;//value to be calculated from the pid controller, public for tuning
    public double i = 0;//value to be calculated from the pid controller, public for tuning
    public double d = 0;//value to be calculated from the pid controller, public for tuning

    public double ppv = 0;//previous process variable
    public double lowerIntegral = -1;//sets limit of intergral variable
    public double upperIntegral = 1;//sets limit of intergral variable
    public double e = 0;//error variable

    public ElapsedTime time = new ElapsedTime();

    /***
     * pidController creates a new PID controller. Could potentially be used for the Gyroscope sensor.
     *
     * @param pGain proportional gain constant
     * @param iGain integral gain constant
     * @param dGain derivative gain constant
     */
    public pidController(double pGain, double iGain, double dGain) {
        this.pGain = pGain;
        this.iGain = iGain;
        this.dGain = dGain;
    }

    public void setSp(double sp) {
        this.sp = sp;
    }

    public void setLowerIntegral(double lowerIntegral) {
        this.lowerIntegral = lowerIntegral;
    }

    public void setUpperIntegral(double upperIntegral) {
        this.upperIntegral = upperIntegral;
    }

    public double getPID(double pv) {
        if (this.ppv == 0)
            this.ppv = pv;

        this.e = (this.sp - pv);
        this.p = this.pGain * e;
        this.i = this.iGain * this.p;

        if (this.i > this.upperIntegral)
            this.i = this.upperIntegral;

        if (this.i < this.lowerIntegral)
            this.i = this.lowerIntegral;

        this.d = -this.dGain * this.ppv * this.time.seconds();
        this.ppv = pv;
        this.time.reset();
        return (this.p + this.i + this.d);
    }
}
