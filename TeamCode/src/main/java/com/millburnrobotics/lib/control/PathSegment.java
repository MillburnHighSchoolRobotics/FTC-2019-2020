package com.millburnrobotics.lib.control;

import com.millburnrobotics.lib.geometry.Pose;

public abstract class PathSegment {
    Pose start,end;

    public abstract Pose get(double s);
    public abstract Pose _get(double r);

    public abstract double length();
    public abstract Pose deriv(double s);
    public abstract Pose secondDeriv(double s);

    public Pose start() {
        return start;
    }
    public Pose end() {
        return end;
    }
}