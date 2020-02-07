package com.millburnrobotics.skystone.paths;

import com.millburnrobotics.lib.control.Path;
import com.millburnrobotics.lib.control.PathBuilder;
import com.millburnrobotics.lib.geometry.Pose;
import com.millburnrobotics.lib.geometry.Waypoint;
import com.millburnrobotics.skystone.Constants;
import com.millburnrobotics.skystone.subsystems.Robot;

import java.util.ArrayList;


public class DetectPath implements PathContainer {
    private int initialBlock;

    public DetectPath(int initialBlock) {
        this.initialBlock = initialBlock;
    }

    @Override
    public Path buildPath() {
        ArrayList<Waypoint> waypoints = new ArrayList<>();

        if (Robot.getInstance().side == Constants.Side.BLUE) {
            if (initialBlock == 3) {

            } else {

            }
            waypoints.add(new Waypoint(Robot.getInstance().getOdometry().getPose(),0));
            waypoints.add(new Waypoint(new Pose(-36,49,Math.PI),0));
        } else {
            waypoints.add(new Waypoint(Robot.getInstance().getOdometry().getPose(),Math.PI));
            waypoints.add(new Waypoint(new Pose(36,49,0),Math.PI));
        }

        return PathBuilder.buildPath(waypoints, 0.1, 0.8, 0.5);
    }
}
