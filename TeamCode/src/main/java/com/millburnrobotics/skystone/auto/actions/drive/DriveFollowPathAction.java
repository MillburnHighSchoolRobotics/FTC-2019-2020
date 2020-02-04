package com.millburnrobotics.skystone.auto.actions.drive;

import com.millburnrobotics.lib.geometry.Pose;
import com.millburnrobotics.skystone.auto.actions.Action;
import com.millburnrobotics.skystone.paths.PathContainer;
import com.millburnrobotics.skystone.subsystems.Robot;

public class DriveFollowPathAction implements Action {

    private PathContainer container;

    public DriveFollowPathAction(PathContainer container) {
        this.container = container;
    }

    @Override
    public void start() {
        Robot.getInstance().getDrive().followPath(container.buildPath());
    }

    @Override
    public void update() {
        Pose current = Robot.getInstance().getOdometry().getPose();
        Robot.getInstance().getDrive().updatePathFollower(current);
    }

    @Override
    public boolean isFinished() {
        return Robot.getInstance().getDrive().isDoneWithPath();
    }

    @Override
    public void done() {
        Robot.getInstance().getDrive().stop();
    }
}
