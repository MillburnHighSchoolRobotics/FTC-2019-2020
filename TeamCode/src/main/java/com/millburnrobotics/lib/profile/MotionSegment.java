package com.millburnrobotics.lib.profile;

public class MotionSegment { // segment of motion with constant acceleration
    private MotionState start;
    double dt;
    public MotionSegment(MotionState start, double dt) {
        this.start = start;
        this.dt = dt;
    }
    public MotionState get(double t) {
        return start.get(t);
    }
    public MotionState end() {
        return start.get(dt);
    }
}
