package com.millburnrobotics.skystone.paths;

import com.millburnrobotics.lib.control.Path;
import com.millburnrobotics.lib.control.PathBuilder;
import com.millburnrobotics.lib.geometry.Pose;
import com.millburnrobotics.lib.geometry.Waypoint;
import com.millburnrobotics.skystone.Constants;
import com.millburnrobotics.skystone.subsystems.Robot;

import java.util.ArrayList;


public class CyclePath implements PathContainer {
    private int block;

    public CyclePath(int block) {
        this.block = block;
    }

    @Override
    public Path buildPath() {
        ArrayList<Waypoint> waypoints = new ArrayList<>();

        if (Robot.getInstance().side == Constants.SIDE.BLUE) {
            waypoints.add(new Waypoint(Robot.getInstance().getOdometry().getPose(),0));
            if (block == 1) {
                waypoints.add(new Waypoint(Constants.AutonConstants.BLUE_BLOCK_1,0));
            } else if (block == 2) {
                waypoints.add(new Waypoint(Constants.AutonConstants.BLUE_BLOCK_2,0));
            } else if (block == 3) {
                waypoints.add(new Waypoint(Constants.AutonConstants.BLUE_BLOCK_3,0));
            } else if (block == 4) {
                waypoints.add(new Waypoint(Constants.AutonConstants.BLUE_BLOCK_4,0));
            } else if (block == 5) {
                waypoints.add(new Waypoint(Constants.AutonConstants.BLUE_BLOCK_5,0));
            } else {
                waypoints.add(new Waypoint(Constants.AutonConstants.BLUE_BLOCK_6,0));
            }
        } else {
            waypoints.add(new Waypoint(Robot.getInstance().getOdometry().getPose(),0));
            if (block == 1) {
                //waypoints.add(new Waypoint(Constants.AutonConstants.RED_BLOCK_1,0)); TODO add RED_BLOCK_1
            } else if (block == 2) {
                waypoints.add(new Waypoint(Constants.AutonConstants.RED_BLOCK_2,0));
            } else if (block == 3) {
                waypoints.add(new Waypoint(Constants.AutonConstants.RED_BLOCK_3,0));
            } else if (block == 4) {
                waypoints.add(new Waypoint(Constants.AutonConstants.RED_BLOCK_4,0));
            } else if (block == 5) {
                waypoints.add(new Waypoint(Constants.AutonConstants.RED_BLOCK_5,0));
            } else {
                waypoints.add(new Waypoint(Constants.AutonConstants.RED_BLOCK_6,0));
            }
        }

        return PathBuilder.buildPath(waypoints, 1, 0.5);
    }
}
