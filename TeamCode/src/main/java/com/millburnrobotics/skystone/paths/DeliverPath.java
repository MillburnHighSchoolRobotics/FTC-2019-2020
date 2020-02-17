package com.millburnrobotics.skystone.paths;

import com.millburnrobotics.lib.control.Path;
import com.millburnrobotics.lib.control.PathBuilder;
import com.millburnrobotics.lib.geometry.Pose;
import com.millburnrobotics.lib.geometry.Waypoint;
import com.millburnrobotics.skystone.Constants;
import com.millburnrobotics.skystone.subsystems.Robot;

import java.util.ArrayList;

import static com.millburnrobotics.skystone.Constants.DriveConstants.BOT_LENGTH;
import static com.millburnrobotics.skystone.Constants.DriveConstants.BOT_WIDTH;
import static com.millburnrobotics.skystone.Constants.FieldConstants.FOUNDATION_LENGTH;
import static com.millburnrobotics.skystone.Constants.SideClawConstants.CLAW_EXTEND;
import static com.millburnrobotics.skystone.Constants.SideClawConstants.CLAW_TO_BACK;


public class DeliverPath implements PathContainer {
    private int pos;

    public DeliverPath(int pos) {
        this.pos = pos;
    }

    @Override
    public Path buildPath() {
        ArrayList<Waypoint> waypoints = new ArrayList<>();

        if (Robot.getInstance().side == Constants.Side.BLUE) {
            double X_BLUE_DELIVERY = -24-BOT_WIDTH/2.0-CLAW_EXTEND;
            double Y_BLUE_DELIVERY = 72-4-FOUNDATION_LENGTH/2.0+(BOT_LENGTH/2.0-CLAW_TO_BACK);

            waypoints.add(new Waypoint(Robot.getInstance().getOdometry().getPose(),Math.toRadians(20)));
            waypoints.add(new Waypoint(new Pose(-38,0,0),0));
            if (pos == 1) {
                waypoints.add(new Waypoint(new Pose(X_BLUE_DELIVERY, Y_BLUE_DELIVERY+4,0),Math.toRadians(320)));
            } else if (pos == 2) {
                waypoints.add(new Waypoint(new Pose(X_BLUE_DELIVERY, Y_BLUE_DELIVERY-6,0),Math.toRadians(320)));
            }
        } else {
            double X_BLUE_DELIVERY = 24+BOT_WIDTH/2.0+CLAW_EXTEND;
            double Y_BLUE_DELIVERY = 72-4-FOUNDATION_LENGTH/2.0+(BOT_LENGTH/2.0-CLAW_TO_BACK);
            waypoints.add(new Waypoint(Robot.getInstance().getOdometry().getPose(),Math.toRadians(340)));
            waypoints.add(new Waypoint(new Pose(38,0,Math.PI),0));
            if (pos == 1) {
                waypoints.add(new Waypoint(new Pose(X_BLUE_DELIVERY, Y_BLUE_DELIVERY+4,Math.PI),Math.toRadians(40)));
            } else if (pos == 2) {
                waypoints.add(new Waypoint(new Pose(X_BLUE_DELIVERY, Y_BLUE_DELIVERY-6,Math.PI),Math.toRadians(40)));
            }
        }

        return PathBuilder.buildPath(waypoints, 0.3, 1, 0.7);
    }
}
