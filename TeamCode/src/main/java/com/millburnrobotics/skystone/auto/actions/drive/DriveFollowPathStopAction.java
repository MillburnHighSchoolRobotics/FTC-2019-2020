package com.millburnrobotics.skystone.auto.actions.drive;

import com.millburnrobotics.lib.control.Path;
import com.millburnrobotics.lib.geometry.Pose;
import com.millburnrobotics.skystone.Robot;
import com.millburnrobotics.skystone.auto.actions.Action;

public class DriveFollowPathStopAction implements Action {

    private Path current_path;
    private double crossY;

    private double minPower, maxPower;

    public DriveFollowPathStopAction(Path path, double crossY, double minPower, double maxPower) {
        this.current_path = path;
        this.crossY = crossY;
        this.minPower = minPower;
        this.maxPower = maxPower;
    }

    @Override
    public void start() {
        Robot.getInstance().getDrive().followPath(current_path);
    }

    @Override
    public void update() {
        Pose current = Robot.getInstance().getOdometry().getPose();
        Robot.getInstance().getDrive().updatePathFollower(current, minPower, maxPower);
    }

    @Override
    public boolean isFinished() {
        return Robot.getInstance().getOdometry().getY() > crossY;
    }

    @Override
    public void done() {
        Robot.getInstance().getDrive().stop();
    }
}
