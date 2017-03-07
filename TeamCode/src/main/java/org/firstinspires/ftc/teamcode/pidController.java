package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by mail2 on 10/27/2016.
 */

/***
 * a class designed for making a pid controller...
 * It could probably be used with the gyroscope sensor to reduce rotation
 */
class pidController {
    //set target for pv
    private double sp = 0;

    private double pGain;
    private double iGain;
    private double dGain;

    //value to be calculated from the pid controller
    double p = 0;
    //value to be calculated from the pid controller
    private double i = 0;
    //value to be calculated from the pid controller
    double d = 0;

    private double ppv = 0;//previous process variable
    private double lowerIntegral = -1;//sets limit of intergral variable
    private double upperIntegral = 1;//sets limit of intergral variable
    private double e = 0;//error variable
    private double error = 0;//tuning error variable

    private boolean phase1 = true;

    //Don't question my spelling...
    private double maxOscilate = 0;
    private double minOscilate = 0;

    private double oscilateTime = 0;
    private boolean tunning = true;

    private ElapsedTime oscilateTimer = new ElapsedTime();
    private ElapsedTime time = new ElapsedTime();
    private ElapsedTime tunertime = new ElapsedTime();

    /***
     * pidController creates a new PID controller. Could potentially be used for the Gyroscope sensor.
     *
     * @param pGain proportional gain constant
     * @param iGain integral gain constant
     * @param dGain derivative gain constant
     */
    pidController(double pGain, double iGain, double dGain) {
        this.pGain = pGain;
        this.iGain = iGain;
        this.dGain = dGain;
    }

    void setSp(double sp) {
        this.sp = sp;
    }

    void setLowerIntegral(double lowerIntegral) {
        this.lowerIntegral = lowerIntegral;
    }

    void setUpperIntegral(double upperIntegral) {
        this.upperIntegral = upperIntegral;
    }

    double getPID(double pv) {
        if (this.ppv == 0)
            this.ppv = pv;

        this.d = this.dGain * (pv - this.ppv) / this.time.seconds();
        this.error = this.sp - pv;
        this.e = this.sp - pv - this.d;
        this.p = this.pGain * e;
        this.i = this.iGain * this.p;

        if(this.d > 0 && !this.phase1) {
            this.oscilateTime += this.oscilateTimer.seconds();
            this.oscilateTime /= 2;
            this.minOscilate = pv - this.sp;
            this.phase1 = true;
            this.oscilateTimer.reset();
        }

        if(this.d < 0 && this.phase1) {
            this.phase1 = false;
            this.maxOscilate = pv - this.sp;
        }

        this.i = Math.min(this.upperIntegral, Math.max(this.lowerIntegral, this.i));

        this.ppv = pv;
        this.time.reset();
        return (this.p + this.i + this.d);
    }

    double getPID(double pv, Telemetry telemetry) {
        if (this.ppv == 0)
            this.ppv = pv;

        this.d = this.dGain * (pv - this.ppv) / this.time.seconds();
        this.error = this.sp - pv;
        this.e = this.sp - pv - this.d;
        this.p = this.pGain * e;
        this.i = this.iGain * this.p;

        this.i = Math.min(this.upperIntegral, Math.max(this.lowerIntegral, this.i));

        this.ppv = pv;
        this.time.reset();
        telemetry.addData("p", "%.2f", this.p);
        telemetry.addData("i", "%.2f", this.i);
        telemetry.addData("d", "%.2f", this.d);
        telemetry.addData("oscilate time", "%.2f", this.oscilateTime);
        telemetry.addData("max oscilate", "%.2f", this.maxOscilate);
        telemetry.addData("min oscilate", "%.2f", this.minOscilate);
        return (this.p + this.i + this.d);
    }

    void pTune() {
        if (this.tunning && tunertime.seconds() > 1) {
            if (this.oscilateTime == 0) {
                this.pGain += 0.01;
                this.sp += Math.random() * 16 - 8;
                tunertime.reset();
            } else if(tunertime.seconds() > 20) {
                this.tunning = false;
                this.iGain = 1.2 * this.p / this.oscilateTime;
                this.dGain = 3 * this.p * this.oscilateTime / 40;
                this.pGain = this.pGain * 0.60;
            }
        }
    }
}
