package com.millburnrobotics.skystone.paths;

import com.millburnrobotics.lib.control.Path;
import com.millburnrobotics.lib.control.PathBuilder;
import com.millburnrobotics.lib.geometry.Pose;
import com.millburnrobotics.lib.geometry.Waypoint;
import com.millburnrobotics.skystone.Constants;
import com.millburnrobotics.skystone.subsystems.Robot;

import java.util.ArrayList;


public class ParkWallPath implements PathContainer {
    @Override
    public Path buildPath() {
        ArrayList<Waypoint> waypoints = new ArrayList<>();

        if (Robot.getInstance().side == Constants.Side.BLUE) {
            waypoints.add(new Waypoint(Robot.getInstance().getOdometry().getPose(),Math.PI));
            waypoints.add(new Waypoint(new Pose(-63, 0, Math.PI),Math.PI));
        } else {
            waypoints.add(new Waypoint(Robot.getInstance().getOdometry().getPose(),Math.PI));
            waypoints.add(new Waypoint(new Pose(63, 0, Math.PI),Math.PI));
        }

        return PathBuilder.buildPath(waypoints, 0.3, 0.8, 0.5);
    }
}
