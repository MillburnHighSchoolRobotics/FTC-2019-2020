package com.millburnrobotics.skystone.auto.actions.drive;

import com.millburnrobotics.lib.control.Path;
import com.millburnrobotics.lib.geometry.Pose;
import com.millburnrobotics.lib.util.MathUtils;
import com.millburnrobotics.skystone.auto.actions.Action;
import com.millburnrobotics.skystone.paths.PathContainer;
import com.millburnrobotics.skystone.subsystems.Robot;
import com.qualcomm.robotcore.util.ElapsedTime;

import static com.millburnrobotics.skystone.Constants.DriveConstants.PATH_HEADING_TIME;
import static com.millburnrobotics.skystone.Constants.DriveConstants.ROTATION_THRESHOLD;
import static com.millburnrobotics.skystone.Constants.DriveConstants.STRAFE_THRESHOLD;
import static com.millburnrobotics.skystone.Constants.DriveConstants.TURN_POWER;

public class DriveFollowPathAction implements Action {

    private PathContainer container;
    private Path current_path;
    private boolean strafe;
    private ElapsedTime rotationTimer;

    public DriveFollowPathAction(PathContainer container) {
        this.container = container;
    }

    @Override
    public void start() {
        strafe = true;
        current_path = container.buildPath();
        Robot.getInstance().getDrive().followPath(current_path);
    }

    @Override
    public void update() {
        if (strafe) {
            Pose current = Robot.getInstance().getOdometry().getPose();
            Robot.getInstance().getDrive().updatePathFollower(current);
        } else {
            if (rotationTimer == null) {
                rotationTimer = new ElapsedTime();
            }
            Robot.getInstance().getDrive().rotateTo(Math.toDegrees(current_path.get(current_path.length()).heading), TURN_POWER);
        }
    }

    @Override
    public boolean isFinished() {
        if (strafe) {
            if (MathUtils.equals(Robot.getInstance().getOdometry().getPose().distTo(current_path.end()),0,STRAFE_THRESHOLD)) {
                strafe = false;
            }
        } else {
            if (rotationTimer.milliseconds() > PATH_HEADING_TIME) {
                return true;
            }
            return MathUtils.equals(Math.toDegrees(current_path.get(current_path.length()).heading), Math.toDegrees(Robot.getInstance().pose.heading), ROTATION_THRESHOLD);
        }
        return false;
    }

    @Override
    public void done() {
        Robot.getInstance().getDrive().stop();
    }
}
