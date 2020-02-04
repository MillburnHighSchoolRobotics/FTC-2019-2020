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

        if (Robot.getInstance().side == Constants.SIDE.BLUE) {
            waypoints.add(new Waypoint(Robot.getInstance().getOdometry().getPose(),Math.PI));
            waypoints.add(new Waypoint(new Pose(-66, 24+Constants.DriveConstants.BOT_WIDTH/2, Math.PI),Math.PI));
        } else {
            waypoints.add(new Waypoint(Robot.getInstance().getOdometry().getPose(),0));
            waypoints.add(new Waypoint(new Pose(66, 24+Constants.DriveConstants.BOT_WIDTH/2, 0),0));
        }

        return PathBuilder.buildPath(waypoints, 1, 0.5);
    }
}
