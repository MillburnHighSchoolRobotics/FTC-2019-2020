package com.millburnrobotics.skystone.paths;

import com.millburnrobotics.lib.control.Path;
import com.millburnrobotics.lib.control.PathBuilder;
import com.millburnrobotics.lib.geometry.Pose;
import com.millburnrobotics.lib.geometry.Waypoint;
import com.millburnrobotics.skystone.Constants;
import com.millburnrobotics.skystone.subsystems.Robot;

import java.util.ArrayList;


public class FoundationPullBackPath implements PathContainer {
    @Override
    public Path buildPath() {
        ArrayList<Waypoint> waypoints = new ArrayList<>();

        if (Robot.getInstance().side == Constants.Side.BLUE) {
            waypoints.add(new Waypoint(Robot.getInstance().getOdometry().getPose(),Math.PI/2));
            waypoints.add(new Waypoint(new Pose(-66, 24+Constants.DriveConstants.BOT_WIDTH/2, Math.PI/2),Math.PI/2));
        } else {
            waypoints.add(new Waypoint(Robot.getInstance().getOdometry().getPose(),3*Math.PI/2));
            waypoints.add(new Waypoint(new Pose(66, 24+Constants.DriveConstants.BOT_WIDTH/2, 3*Math.PI/2),3*Math.PI/2));
        }

        return PathBuilder.buildPath(waypoints, 0.3, 1, 0.5);
    }
}
