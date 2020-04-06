package com.millburnrobotics.skystone.auto.actions.drive;

import com.millburnrobotics.lib.control.Path;
import com.millburnrobotics.lib.geometry.Pose;
import com.millburnrobotics.skystone.Robot;
import com.millburnrobotics.skystone.auto.actions.Action;

public class DriveFollowPathAction implements Action {

    private Path path;
    public DriveFollowPathAction(Path path) {
        this.path = path;
    }
    @Override
    public void start() {
    }

    @Override
    public void update() {
        Pose current = Robot.getInstance().getOdometry().getPose();
        Pose currentVel = Robot.getInstance().getOdometry().getVelocity();
        Robot.getInstance().getDrive().updatePathFollower(current, currentVel, path);
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
