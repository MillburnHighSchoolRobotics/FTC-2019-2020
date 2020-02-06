package com.millburnrobotics.lib.control;

import com.millburnrobotics.lib.math.MathUtils;

public class MotionProfile {
//    private double A;
    private double maxPower;
    private double s;
    private double k;
    public MotionProfile(double maxPower, double s, double k) {
//        this.A = maxPower/2.0;
        this.maxPower = maxPower;
        this.s = s;
        this.k = k;
    }
    public double getPower(double d) {
//        if (d >= 0 && d < ((1-k)/2.0)*s) {
//            return MathUtils.map(d,0,((1-k)/2.0)*s,0,maxPower);
//        } else if (d > (((1-k)/2.0)+k)*s && d <= s) {
//            return MathUtils.map(d,(((1-k)/2.0)+k)*s,s,maxPower,0);
//        } else {
//            return maxPower;
//        }
//        if (d >= 0 && d < (k*s)) {
//            return -A*Math.cos(Math.PI*(1.0/(s*k))*d)+A;
//        } else if (d > (k*s) && d <= s) {
//            return -A*Math.cos(Math.PI*(1.0/(s*(1-k)))*(d-s))+A;
//        }
        return 0.4;
    }
}
