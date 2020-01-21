package com.millburnrobotics.lib.control;

import com.millburnrobotics.lib.math.Pose;

public class PathGenerator {
    public Path path;
    Pose start;
    public PathGenerator(Pose start) {
        this.path = new Path();
        this.start = start;
    }
    public PathGenerator strafeTo(Pose lineEnd) {
        Pose lineStart = (path.size() == 0) ? start : path.end();
        path.add(new Line(lineStart,lineEnd));
        return this;
    }
    public PathGenerator splineTo(Pose splineEnd) {
        Pose splineStart = (path.size() == 0) ? start : path.end();
        path.add(new Line(splineStart,splineEnd));
        return this;
    }
}