package com.millburnrobotics.lib.control;

import com.millburnrobotics.lib.geometry.Pose;
import com.millburnrobotics.lib.geometry.Waypoint;

import java.util.List;

public class PathBuilder {
    public static Path buildPath(List<Waypoint> w, double maxPower) {
        return buildPath(w, maxPower, 0.5);
    }
    public static Path buildPath(List<Waypoint> w, double maxPower, double k) {
        Path p = new Path();
        if (w.size() < 2) {
            throw new Error("Path must contain at least 2 waypoints");
        } else {
            for (int i = 0; i < w.size()-1; i++) {
                Waypoint w1 = w.get(i);
                Waypoint w2 = w.get(i+1);
                Pose splineStart = new Pose(w1.pose.vec()[0],w1.pose.vec()[1],w1.heading);
                Pose splineEnd = new Pose(w2.pose.vec()[0],w2.pose.vec()[1],w2.heading);
                p.add(new QuinticHermiteSpline(splineStart,splineEnd));
            }
        }
        p.setMotionProfile(maxPower,k);
        return p;
    }
}