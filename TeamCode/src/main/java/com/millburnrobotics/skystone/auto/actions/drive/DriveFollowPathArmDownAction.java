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

public class DriveFollowPathArmDownAction implements Action {

    private PathContainer container;
    private Path current_path;
    private boolean strafe;
    private ElapsedTime rotationTimer;
    private ElapsedTime lastInThreshTimer;
    private double lastTime;
    private double crossY;
    private boolean cross;

    public DriveFollowPathArmDownAction(PathContainer container, double crossY) {
        this.container = container;
        this.crossY = crossY;
    }

    @Override
    public void start() {
        strafe = true;
        cross = false;
        current_path = container.buildPath();
        lastInThreshTimer = new ElapsedTime();
        lastTime = -1;
        Robot.getInstance().getDrive().followPath(current_path);
    }

    @Override
    public void update() {
        Pose current = Robot.getInstance().getOdometry().getPose();
        if ((current.y < crossY) && !cross) {
            cross = true;
            Robot.getInstance().getSideClaw().armDown();
        }
        if (strafe) {
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
                if (lastTime == -1) {
                    lastTime = lastInThreshTimer.milliseconds();
                } else if (lastInThreshTimer.milliseconds() - lastTime > 200) {
                    strafe = false;
                }
            } else {
                lastTime = -1;
            }
        } else {
            if (rotationTimer.milliseconds() > PATH_HEADING_TIME) {
                return true;
            }
            double currentHeading = Math.toDegrees(Robot.getInstance().getOdometry().getPose().getHeading());
            double targetHeading = Math.toDegrees(current_path.get(current_path.length()).heading);
            if (currentHeading - targetHeading > 180) {
                currentHeading -= 360;
            } else if (targetHeading - currentHeading > 180) {
                currentHeading += 360;
            }
            return MathUtils.equals(targetHeading, currentHeading, ROTATION_THRESHOLD);
        }
        return false;
    }

    @Override
    public void done() {
        Robot.getInstance().getDrive().stop();
    }
}