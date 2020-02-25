package com.millburnrobotics.lib.control;

import com.millburnrobotics.lib.geometry.Pose;
import com.millburnrobotics.lib.geometry.Waypoint;

import java.util.List;

public class PathBuilder {
    public static Path buildPath(List<Waypoint> w) {
        Path p = new Path();
        if (w.size() < 2) {
            throw new Error("Path must contain at least 2 waypoints");
        } else {
            for (int i = 0; i < w.size()-1; i++) {
                Waypoint w1 = w.get(i);
                Waypoint w2 = w.get(i+1);
                Pose splineStart = new Pose(w1.pose.vec()[0],w1.pose.vec()[1],w1.bearing);
                Pose splineEnd = new Pose(w2.pose.vec()[0],w2.pose.vec()[1],w2.bearing);
                p.add(new QuinticHermiteSpline(splineStart,splineEnd));
            }
        }

        p.startHeading = w.get(0).pose.heading;
        p.endHeading = w.get(w.size()-1).pose.heading;
        if (p.startHeading - p.endHeading > Math.PI) {
            p.startHeading -= 2*Math.PI;
        } else if (p.endHeading - p.startHeading > Math.PI) {
            p.startHeading += 2*Math.PI;
        }

        return p;
    }
}