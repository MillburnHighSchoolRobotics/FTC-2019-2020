package com.millburnrobotics.skystone.auto.actions.drive;

import com.millburnrobotics.lib.control.Path;
import com.millburnrobotics.lib.geometry.Pose;
import com.millburnrobotics.skystone.Robot;
import com.millburnrobotics.skystone.auto.actions.Action;

public class DriveFollowPathArmDownAction implements Action {

    private Path path;
    private double crossY;
    private boolean cross;
    public DriveFollowPathArmDownAction(Path path, double crossY) {
        this.path = path;
        this.crossY = crossY;
    }
    @Override
    public void start() {
        cross = false;
    }

    @Override
    public void update() {
        Pose current = Robot.getInstance().getOdometry().getPose();
        Pose currentVel = Robot.getInstance().getOdometry().getVelocity();
        Robot.getInstance().getDrive().updatePathFollower(current, currentVel, path);
        if ((current.y < crossY) && !cross) {
            cross = true;
            Robot.getInstance().getSideClaw().armMid();
        }
    }

    @Override
    public boolean isFinished() {
        return Robot.getInstance().getDrive().getLookahead().equals(path.end());
    }

    @Override
    public void done() {
        Robot.getInstance().getDrive().stop();
    }
}