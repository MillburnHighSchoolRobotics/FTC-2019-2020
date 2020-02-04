package com.millburnrobotics.skystone.paths;

import com.millburnrobotics.lib.control.Path;
import com.millburnrobotics.lib.control.PathBuilder;
import com.millburnrobotics.lib.geometry.Pose;
import com.millburnrobotics.lib.geometry.Waypoint;
import com.millburnrobotics.skystone.subsystems.Robot;

import java.util.ArrayList;


public class ToFoundation implements PathContainer {
    @Override
    public Path buildPath(boolean blueSide) {
        ArrayList<Waypoint> waypoints = new ArrayList<>();

        if (blueSide) {
            waypoints.add(new Waypoint(Robot.getInstance().getOdometry().getPose(),0));
            waypoints.add(new Waypoint(new Pose(24,24,90),0));
        }

        return PathBuilder.buildPath(waypoints, 1, 0.4);
    }
}
