package com.millburnrobotics.skystone.test.lib;

import com.millburnrobotics.lib.control.Path;
import com.millburnrobotics.lib.control.PathBuilder;
import com.millburnrobotics.lib.followers.AdaptivePurePursuitFollower;
import com.millburnrobotics.lib.geometry.Pose;
import com.millburnrobotics.lib.geometry.Waypoint;
import com.millburnrobotics.lib.util.MathUtils;

import java.util.ArrayList;

public class PurePursuitTest {
    public static void main(String[] args) {
        ArrayList<Waypoint> waypoints = new ArrayList<>();
        waypoints.add(new Waypoint(new Pose(0,0,0),0));
        waypoints.add(new Waypoint(new Pose(48,48,0),0));
        Path path =  PathBuilder.buildPath(waypoints);
        AdaptivePurePursuitFollower follower = new AdaptivePurePursuitFollower(path);

        double variance = 0.2;
        double num = 3;
        Pose p = new Pose(0,0);
        while (!MathUtils.equals(p.distTo(path.end()),0,1)) {
            Pose pose1 = follower.getLookaheadPoint(p);
            System.out.println("(x-" + p.x + ")^{2}+(y-" + p.y + ")^{2}=1");
            if (pose1.equals(path.end())) {
                break;
            }
            double multiplier = (Math.random()*variance)+1-variance;
            p = new Pose(p.x+(Math.abs(pose1.x-p.x)/num)*multiplier,p.y+(Math.abs(pose1.y-p.y)/num)*multiplier);
        }
    }
}
