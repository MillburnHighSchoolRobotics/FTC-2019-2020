package com.millburnrobotics.skystone.auto.actions.drive;

import com.millburnrobotics.lib.control.Path;
import com.millburnrobotics.lib.geometry.Pose;
import com.millburnrobotics.lib.util.MathUtils;
import com.millburnrobotics.skystone.auto.actions.Action;
import com.millburnrobotics.skystone.paths.PathContainer;
import com.millburnrobotics.skystone.subsystems.Robot;
import com.qualcomm.robotcore.util.ElapsedTime;

import static com.millburnrobotics.skystone.Constants.DriveConstants.STRAFE_THRESHOLD;

public class DriveTimedFollowPathAction implements Action {

    private PathContainer container;
    private Path current_path;
    private double time;
    private ElapsedTime timer;

    public DriveTimedFollowPathAction(PathContainer container, double time) {
        this.container = container;
        this.time = time;
    }

    @Override
    public void start() {
        current_path = container.buildPath();
        Robot.getInstance().getDrive().followPath(current_path);
        this.timer = new ElapsedTime();
    }

    @Override
    public void update() {
        Pose current = Robot.getInstance().getOdometry().getPose();
        Robot.getInstance().getDrive().updatePathFollower(current);
    }

    @Override
    public boolean isFinished() {
        if (this.timer.milliseconds() >= time) {
            return true;
        }
        return MathUtils.equals(Robot.getInstance().getOdometry().getPose().distTo(current_path.end()),0,STRAFE_THRESHOLD);
    }

    @Override
    public void done() {
        Robot.getInstance().getDrive().stop();
    }
}
