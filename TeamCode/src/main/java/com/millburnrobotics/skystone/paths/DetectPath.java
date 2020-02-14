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


public class DetectPath implements PathContainer {
    @Override
    public Path buildPath() {
        ArrayList<Waypoint> waypoints = new ArrayList<>();
        if (Robot.getInstance().side == Constants.Side.BLUE) {
            double X_BLUE_BLOCK_CLAW = -24-BOT_WIDTH/2.0-CLAW_EXTEND;
            double Y_BLUE_BLOCK_CLAW = -24-4+(BOT_LENGTH/2.0-CLAW_TO_BACK);
            if (Robot.getInstance().block == Constants.Block.CENTER) {
                waypoints.add(new Waypoint(Robot.getInstance().getOdometry().getPose(),3*Math.PI/2));
                waypoints.add(new Waypoint(new Pose(-72+ BOT_WIDTH/2.0+4,-48+ BOT_LENGTH/2.0,0),3*Math.PI/2));
                waypoints.add(new Waypoint(new Pose(X_BLUE_BLOCK_CLAW,Y_BLUE_BLOCK_CLAW-32,0),3*Math.PI/2));
            } else if (Robot.getInstance().block == Constants.Block.LEFT || Robot.getInstance().block == Constants.Block.NULL) {
                waypoints.add(new Waypoint(Robot.getInstance().getOdometry().getPose(),3*Math.PI/2));
                waypoints.add(new Waypoint(new Pose(-72+ BOT_WIDTH/2.0+4,-48+ BOT_LENGTH/2.0,0),3*Math.PI/2));
                waypoints.add(new Waypoint(new Pose(X_BLUE_BLOCK_CLAW,Y_BLUE_BLOCK_CLAW-24,0),3*Math.PI/2));
            } else if (Robot.getInstance().block == Constants.Block.RIGHT) {
                waypoints.add(new Waypoint(Robot.getInstance().getOdometry().getPose(),3*Math.PI/2));
                waypoints.add(new Waypoint(new Pose(-72+ BOT_WIDTH/2.0+4,-48+ BOT_LENGTH/2.0,0),3*Math.PI/2));
                waypoints.add(new Waypoint(new Pose(X_BLUE_BLOCK_CLAW,Y_BLUE_BLOCK_CLAW-16,0),3*Math.PI/2));
            }
        } else {

        }

        return PathBuilder.buildPath(waypoints, 0.25, 0.7, 0.6);
    }
}
