package com.millburnrobotics.skystone.auto.actions.drive;

import com.millburnrobotics.lib.control.Path;
import com.millburnrobotics.lib.geometry.Pose;
import com.millburnrobotics.lib.math.MathUtils;
import com.millburnrobotics.skystone.auto.actions.Action;
import com.millburnrobotics.skystone.paths.PathContainer;
import com.millburnrobotics.skystone.subsystems.Robot;

public class DriveFollowPathAction implements Action {

    private PathContainer container;
    private Path current_path;

    public DriveFollowPathAction(PathContainer container) {
        this.container = container;
    }

    @Override
    public void start() {
        current_path = container.buildPath();
        Robot.getInstance().getDrive().followPath(current_path);
    }

    @Override
    public void update() {
        Pose current = Robot.getInstance().getOdometry().getPose();
        Robot.getInstance().getDrive().updatePathFollower(current);
    }

    @Override
    public boolean isFinished() {
        return MathUtils.equals(Robot.getInstance().getOdometry().getPose().distTo(current_path.end()),0,Robot.getInstance().getDrive().strafeThreshold);
    }

    @Override
    public void done() {
        Robot.getInstance().getDrive().stop();
    }
}
