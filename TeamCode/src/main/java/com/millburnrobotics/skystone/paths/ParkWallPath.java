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

        if (Robot.getInstance().side == Constants.SIDE.BLUE) {
            waypoints.add(new Waypoint(Robot.getInstance().getOdometry().getPose(),3*Math.PI/2));
            waypoints.add(new Waypoint(new Pose(-63, 0, Math.PI),3*Math.PI/2));
        } else {
            waypoints.add(new Waypoint(Robot.getInstance().getOdometry().getPose(),3*Math.PI/2));
            waypoints.add(new Waypoint(new Pose(63, 0, 0),3*Math.PI/2));
        }

        return PathBuilder.buildPath(waypoints, 1, 0.5);
    }
}
