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
    //value to be calculated from the pid controller
    private double p = 0;
    //value to be calculated from the pid controller
    private double d = 0;
    //set target for pv
    double sp = 0;
    double pGain;
    double iGain;
    double dGain;
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

        d = dGain * (ppv - pv) / time.seconds();
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
        if(d > 0 && !phase1) {
            oscilateTime += oscilateTimer.seconds();
            oscilateTime /= 2;
            minOscilate = pv - sp;
            phase1 = true;
            oscilateTimer.reset();
        }

        if(d < 0 && phase1) {
            phase1 = false;
            maxOscilate = pv - sp;
        }

        d = dGain * (ppv - pv) / time.seconds();
        e = sp - pv;
        p = pGain * e;
        i = iGain * p;

        i = Math.min(upperIntegral, Math.max(lowerIntegral, i));

        ppv = pv;
        time.reset();
        telemetry.addData("p", "%.2f", p);
        telemetry.addData("i", "%.2f", i);
        telemetry.addData("d", "%.2f", d);
        telemetry.addData("e", "%.2f", e);
        telemetry.addData("pGain", "%.2f", pGain);
        telemetry.addData("iGain", "%.2f", iGain);
        telemetry.addData("dGain", "%.2f", dGain);
        telemetry.addData("oscilate time", "%.2f", oscilateTime);
        telemetry.addData("max oscilate", "%.2f", maxOscilate);
        telemetry.addData("min oscilate", "%.2f", minOscilate);
        return (p + i + d);
    }

    void pTune() {
        //https://en.wikipedia.org/wiki/Ziegler%E2%80%93Nichols_method
        if (tunning && tunertime.seconds() > 1) {
            if (oscilateTime == 0) {
                pGain += 0.01;
                sp += Math.random() * 32 - 16;
                tunertime.reset();
            } else if(tunertime.seconds() > 20) {
                tunning = false;
                iGain = 1.2 * p / oscilateTime;
                dGain = 3 * p * oscilateTime / 40;
                pGain = pGain * 0.60;
            }
        }
    }
}
