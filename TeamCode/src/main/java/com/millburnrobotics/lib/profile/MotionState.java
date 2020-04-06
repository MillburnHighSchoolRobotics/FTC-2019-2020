package com.millburnrobotics.lib.profile;

public class MotionState {
    public final double x;
    public final double v;
    public final double a;
    public final double j;
    public MotionState() {
        this.x = 0;
        this.v = 0;
        this.a = 0;
        this.j = 0;
    }
    public MotionState(double x, double v) {
        this.x = x;
        this.v = v;
        this.a = 0;
        this.j = 0;
    }
    public MotionState(double x, double v, double a) {
        this.x = x;
        this.v = v;
        this.a = a;
        this.j = 0;
    }
    public MotionState(double x, double v, double a, double j) {
        this.x = x;
        this.v = v;
        this.a = a;
        this.j = j;
    }

    public MotionState(MotionState state) {
        this(state.x, state.v, state.a, state.j);
    }
    public MotionState get(double t) {
        return new MotionState(
                x + v * t + a / 2.0 * t * t + j / 6.0 * t * t * t,
                v + a * t + j / 2.0 * t * t,
                a + j * t,
                j
        );
    }

    @Override
    public String toString() {
        return "(x=" + x + ", v=" + v + ", a=" + a + ", j=" + j + ")";
    }
}
