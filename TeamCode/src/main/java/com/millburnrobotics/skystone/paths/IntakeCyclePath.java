package com.millburnrobotics.skystone.paths;

import com.millburnrobotics.lib.control.Path;
import com.millburnrobotics.lib.control.PathBuilder;
import com.millburnrobotics.lib.geometry.Pose;
import com.millburnrobotics.lib.geometry.Waypoint;
import com.millburnrobotics.skystone.Constants;
import com.millburnrobotics.skystone.subsystems.Robot;

import java.util.ArrayList;


public class IntakeCyclePath implements PathContainer {
    @Override
    public Path buildPath() {
        ArrayList<Waypoint> waypoints = new ArrayList<>();

        if (Robot.getInstance().side == Constants.Side.BLUE) {
            waypoints.add(new Waypoint(Robot.getInstance().getOdometry().getPose(),Math.toRadians(180)));
            waypoints.add(new Waypoint(new Pose(-40,36,Math.PI),Math.toRadians(180)));
            waypoints.add(new Waypoint(new Pose(-40,-24,Math.PI),Math.toRadians(180)));
            waypoints.add(new Waypoint(new Pose(-24, -54, Math.PI), Math.toRadians(270)));
        } else {
            waypoints.add(new Waypoint(Robot.getInstance().getOdometry().getPose(),0));
        }

        return PathBuilder.buildPath(waypoints, 0.25, 0.9, 0.5);
    }
}
