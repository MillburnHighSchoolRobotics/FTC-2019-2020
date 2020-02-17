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
import static com.millburnrobotics.skystone.Constants.SideClawConstants.CLAW_EXTEND;
import static com.millburnrobotics.skystone.Constants.SideClawConstants.CLAW_TO_BACK;


public class CyclePath implements PathContainer {
    private int pos;

    public CyclePath(int pos) {
        this.pos = pos;
    }
    public CyclePath() {
        if (Robot.getInstance().side == Constants.Side.BLUE) {
            if (Robot.getInstance().block == Constants.Block.LEFT) {
                this.pos = 1;
            } else if (Robot.getInstance().block == Constants.Block.CENTER) {
                this.pos = 2;
            } else if (Robot.getInstance().block == Constants.Block.RIGHT) {
                this.pos = 3;
            } else if (Robot.getInstance().block == Constants.Block.NULL) {
                this.pos = 2;
            }
        } else {
            if (Robot.getInstance().block == Constants.Block.RIGHT) {
                this.pos = 1;
            } if (Robot.getInstance().block == Constants.Block.CENTER) {
                this.pos = 2;
            } else {
                this.pos = 3;
            }
        }
    }

    @Override
    public Path buildPath() {
        ArrayList<Waypoint> waypoints = new ArrayList<>();

        if (Robot.getInstance().side == Constants.Side.BLUE) {
            double X_BLUE_BLOCK_CLAW = -24-BOT_WIDTH/2.0-CLAW_EXTEND;
            double Y_BLUE_BLOCK_CLAW = -24-4+(BOT_LENGTH/2.0-CLAW_TO_BACK);

            waypoints.add(new Waypoint(Robot.getInstance().getOdometry().getPose(),Math.toRadians(170)));
            if (pos == 1) {
                waypoints.add(new Waypoint(new Pose(-40,0,0),Math.toRadians(190)));
                waypoints.add(new Waypoint(new Pose(X_BLUE_BLOCK_CLAW, Y_BLUE_BLOCK_CLAW,0),Math.toRadians(220)));
            } else if (pos == 2) {
                waypoints.add(new Waypoint(new Pose(-40,0,0),Math.toRadians(190)));
                waypoints.add(new Waypoint(new Pose(X_BLUE_BLOCK_CLAW,Y_BLUE_BLOCK_CLAW-8,0),Math.toRadians(220)));
            }
        } else {
            double X_BLUE_BLOCK_CLAW = 24+BOT_WIDTH/2.0+CLAW_EXTEND;
            double Y_BLUE_BLOCK_CLAW = -24-4-(BOT_LENGTH/2.0-CLAW_TO_BACK);

            waypoints.add(new Waypoint(Robot.getInstance().getOdometry().getPose(),Math.toRadians(190)));
            if (pos == 1) {
                waypoints.add(new Waypoint(new Pose(40,0,Math.PI),Math.toRadians(170)));
                waypoints.add(new Waypoint(new Pose(X_BLUE_BLOCK_CLAW, Y_BLUE_BLOCK_CLAW,Math.PI),Math.toRadians(140)));
            } else if (pos == 2) {
                waypoints.add(new Waypoint(new Pose(40,0,Math.PI),Math.toRadians(170)));
                waypoints.add(new Waypoint(new Pose(X_BLUE_BLOCK_CLAW,Y_BLUE_BLOCK_CLAW-8,Math.PI),Math.toRadians(140)));
            }
        }

        return PathBuilder.buildPath(waypoints, 0.25, 0.8, 0.7);
    }
}
