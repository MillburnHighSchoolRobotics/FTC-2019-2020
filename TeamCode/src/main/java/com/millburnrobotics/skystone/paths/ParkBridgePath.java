package com.millburnrobotics.skystone.paths;

import com.millburnrobotics.lib.control.Path;
import com.millburnrobotics.lib.control.PathBuilder;
import com.millburnrobotics.lib.geometry.Pose;
import com.millburnrobotics.lib.geometry.Waypoint;
import com.millburnrobotics.skystone.Constants;
import com.millburnrobotics.skystone.subsystems.Robot;

import java.util.ArrayList;


public class ParkBridgePath implements PathContainer {
    @Override
    public Path buildPath() {
        ArrayList<Waypoint> waypoints = new ArrayList<>();

        if (Robot.getInstance().side == Constants.SIDE.BLUE) {
            waypoints.add(new Waypoint(Robot.getInstance().getOdometry().getPose(),3*Math.PI/2));
            waypoints.add(new Waypoint(new Pose(-63, 12),3*Math.PI/2));
            waypoints.add(new Waypoint(new Pose(-36, 0),0));
        } else {
            waypoints.add(new Waypoint(Robot.getInstance().getOdometry().getPose(),3*Math.PI/2));
            waypoints.add(new Waypoint(new Pose(63, 12),3*Math.PI/2));
            waypoints.add(new Waypoint(new Pose(36, 0),Math.PI));
        }

        return PathBuilder.buildPath(waypoints, 1, 0.5);
    }
}
