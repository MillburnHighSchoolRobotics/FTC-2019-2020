package com.millburnrobotics.skystone.paths;

import com.millburnrobotics.lib.control.Path;
import com.millburnrobotics.lib.control.PathBuilder;
import com.millburnrobotics.lib.geometry.Pose;
import com.millburnrobotics.lib.geometry.Waypoint;
import com.millburnrobotics.skystone.Constants;
import com.millburnrobotics.skystone.subsystems.Robot;

import java.util.ArrayList;


public class DeliverPath implements PathContainer {
    private int pos;

    public DeliverPath(int pos) {
        this.pos = pos;
    }

    @Override
    public Path buildPath() {
        ArrayList<Waypoint> waypoints = new ArrayList<>();

        if (Robot.getInstance().side == Constants.SIDE.BLUE) {
            waypoints.add(new Waypoint(Robot.getInstance().getOdometry().getPose(),0));
            if (pos == 1) {
                waypoints.add(new Waypoint(Constants.AutonConstants.BLUE_DELIVERY_1,0));
            } else if (pos == 2) {
                waypoints.add(new Waypoint(Constants.AutonConstants.BLUE_DELIVERY_2,0));
            } else if (pos == 3) {
                waypoints.add(new Waypoint(Constants.AutonConstants.BLUE_DELIVERY_3,0));
            } else {
                waypoints.add(new Waypoint(Constants.AutonConstants.BLUE_DELIVERY_4,0));
            }
        } else {
            waypoints.add(new Waypoint(Robot.getInstance().getOdometry().getPose(),0));
            if (pos == 1) {
                waypoints.add(new Waypoint(Constants.AutonConstants.RED_DELIVERY_1,0));
            } else if (pos == 2) {
                waypoints.add(new Waypoint(Constants.AutonConstants.RED_DELIVERY_2,0));
            } else if (pos == 3) {
                waypoints.add(new Waypoint(Constants.AutonConstants.RED_DELIVERY_3,0));
            } else {
                waypoints.add(new Waypoint(Constants.AutonConstants.RED_DELIVERY_4,0));
            }
        }

        return PathBuilder.buildPath(waypoints, 1, 0.5);
    }
}
