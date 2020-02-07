package com.millburnrobotics.lib.control;

import com.millburnrobotics.lib.util.MathUtils;
import com.millburnrobotics.lib.geometry.Pose;

public class Line extends PathSegment {
    public Line(Pose start, Pose end) {
        this.start = start;
        this.end = end;
    }
    @Override
    public Pose get(double s) {
        return new Pose(
                MathUtils.map(s,0,length(),start.x,end.x),
                MathUtils.map(s,0,length(),start.y,end.y)
        );
    }
    @Override
    public Pose _get(double s) {
        return new Pose(
                MathUtils.map(s,0,1,start.x,end.x),
                MathUtils.map(s,0,1,start.y,end.y)
        );
    }
    @Override
    public double length() {
        return start.minus(end).norm();
    }
    @Override
    public Pose deriv(double s) {
        return end.minus(start).div(length());
    }
    @Override
    public Pose secondDeriv(double s) {
        return new Pose();
    }

}