package com.millburnrobotics.lib.control;

import com.millburnrobotics.lib.geometry.Pose;

public abstract class PathSegment {
    Pose start,end;

    abstract Pose get(double s);
    abstract Pose _get(double r);

    abstract double length();
    abstract Pose deriv(double s);
    abstract Pose secondDeriv(double s);

    public Pose start() {
        return start;
    }
    public Pose end() {
        return end;
    }
}