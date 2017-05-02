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
    double sp = 0;
    double pGain;
    double iGain;
    double dGain;
    boolean tunning = false;
    //value to be calculated from the pid controller
    private double p = 0;
    //value to be calculated from the pid controller
    private double d = 0;
    //value to be calculated from the pid controller
    private double i = 0;
    private double ppv = 0;//previous process variable
    private double lowerIntegral = -1;//sets limit of intergral variable
    private double upperIntegral = 1;//sets limit of intergral variable
    private double e = 0;//error variable
    private boolean phase1 = true;
    //Don't question my spelling...
    private double maxOscilate = 0;
    private double minOscilate = 0;
    private double oscilateTime = 0;
    private ElapsedTime oscilateTimer = new ElapsedTime();
    private ElapsedTime time = new ElapsedTime();
    private ElapsedTime tunertime = new ElapsedTime();
    private ElapsedTime spTime = new ElapsedTime();

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
        if (dGain == 1)
            tunning = true;
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

        d = dGain * (ppv - pv) / time.milliseconds();
        e = sp - pv;
        p = pGain * e;
        i = iGain * p;

        i = Math.min(upperIntegral, Math.max(lowerIntegral, i));

        ppv = pv;
        time.reset();
        return (p + i + d);
    }

    double getPID(double pv, Telemetry telemetry) {

        //detect oscilation
        if (d > 0 && !phase1) {
            oscilateTime = oscilateTimer.milliseconds();
            minOscilate = pv - sp;
            phase1 = true;
            oscilateTimer.reset();
        }

        if (d < 0 && phase1) {
            phase1 = false;
            maxOscilate = pv - sp;
        }

        d = dGain * (ppv - pv) / time.milliseconds();
        e = sp - pv;
        p = pGain * e;
        i = iGain * p;

        i = Math.min(upperIntegral, Math.max(lowerIntegral, i));

        ppv = pv;
        time.reset();
        telemetry.addData("p", "%.4f", p);
        telemetry.addData("i", "%.4f", i);
        telemetry.addData("d", "%.4f", d);
        telemetry.addData("e", "%.4f", e);
        telemetry.addData("pGain", "%.4f", pGain);
        telemetry.addData("iGain", "%.4f", iGain);
        telemetry.addData("dGain", "%.4f", dGain);
        telemetry.addData("oscilate time", "%.4f", oscilateTime);
        telemetry.addData("max oscilate", "%.4f", maxOscilate);
        telemetry.addData("min oscilate", "%.4f", minOscilate);
        if (tunning)
            return p;
        else
            return (p + i + d);
    }

    void pTune() {
        //https://en.wikipedia.org/wiki/Ziegler%E2%80%93Nichols_method
        if (tunning && tunertime.seconds() > 1) {
            if (oscilateTime == 0 || oscilateTime > 3000) {
                pGain += 0.0001;
                if (Math.abs(e) < 5) {
                    if (spTime.seconds() > 10) {
                        sp = Math.random() * 45;
                        phase1 = true;
                        oscilateTimer.reset();
                    }
                } else {
                    spTime.reset();
                }
                tunertime.reset();
            } else if (tunertime.seconds() > 20) {
                tunning = false;
                iGain = 1.2 * p / oscilateTime;
                dGain = 3 * p * oscilateTime / 40;
                pGain = pGain * 0.60;
            }
        }
    }
}
