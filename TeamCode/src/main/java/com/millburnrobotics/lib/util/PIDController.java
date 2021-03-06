package com.millburnrobotics.lib.util;

import android.util.Log;
import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDController {
    private double kP;
    private double kI;
    private double kD;
    private double threshold;
    private boolean switchSign = false;
    private double maxI = Double.MAX_VALUE;

    private double P;
    private double I;
    private double D;
    private ElapsedTime time;

    private double target;

    public PIDController() {
        kP = 0;
        kI = 0;
        kD = 0;
        threshold = 0;

        P = 0;
        I = 0;
        D = 0;
    }

    public PIDController(double kP, double kI, double kD) {

        this();

        this.kP = kP;
        this.kI = kI;
        this.kD = kD;

        threshold = 0;
    }

    public PIDController(double kP, double kI, double kD, double threshold) {
        this(kP, kI, kD);

        this.threshold = threshold;
    }

    public PIDController(double kP, double kI, double kD, double threshold, double target) {
        this(kP, kI, kD, threshold);

        this.target = target;
    }

    public PIDController(double kP, double kI, double kD, double threshold, double target, boolean switchSign) {
        this(kP,kI,kD,threshold,target);
        this.switchSign = switchSign;
    }

    public PIDController(double kP, double kI, double kD, double threshold, double target, boolean switchSign, double maxValue) {
        this(kP,kI,kD,threshold,target,switchSign);
        this.maxI = maxValue;
    }

    public double getPIDOutput(double currentValue) {
        boolean first = false;
        if (time == null) {
            time = new ElapsedTime();
            first = true;
        } else {
            D = ((target - currentValue) - P)/time.seconds();
        }
        P = target - currentValue;
        Log.d("PIDValue: Error", Double.toString(P));



        if (!switchSign ? Math.abs(currentValue - target) < threshold : Math.abs(currentValue - target) > threshold) {
            if (time == null || first) {
                time = new ElapsedTime();
            } else {
                I = time.seconds() * P + I;
                time.reset();
            }
        } else {
            I = 0;
        }

        I = Math.min(I,maxI);

        return kP * P + kI * I + kD * D;
    }

    public void setTarget(double target) {
        this.target = target;
    }

    public double getTarget() {
        return target;
    }

    public void setThreshold(double threshold) {
        this.threshold = threshold;
    }

    public void setKP(double kP) {
        this.kP = kP;
    }

    public void setKI(double kI) {
        this.kI = kI;
    }

    public void setKD(double kD) {
        this.kD = kD;
    }

    public double getKp() {
        return kP;
    }
    public String toString() {
        return P + " " + I + " " + D;
    }
}