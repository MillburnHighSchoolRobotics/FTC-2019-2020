package com.millburnrobotics.lib.control;

public class MotionProfile {
    private double A;
    private double s;
    private double k;
    public MotionProfile(double maxPower, double s, double k) {
        this.A = maxPower/2.0;
        this.s = s;
        this.k = k;
    }
    public double getPower(double d) {
        if (d >= 0 && d < (k*s)) {
            return -A*Math.cos(Math.PI*(1.0/(s*k))*d)+A;
        } else if (d > (k*s) && d <= s) {
            return -A*Math.cos(Math.PI*(1.0/(s*(1-k)))*(d-s))+A;
        }
        return 0;
    }
}
