package com.millburnrobotics.skystone.auto.actions.drive;

import com.millburnrobotics.lib.geometry.Pose;
import com.millburnrobotics.skystone.auto.actions.Action;
import com.millburnrobotics.skystone.paths.PathContainer;
import com.millburnrobotics.skystone.subsystems.Robot;
import com.qualcomm.robotcore.util.ElapsedTime;

public class DriveTimedFollowPathAction implements Action {

    private PathContainer container;
    private double time;
    private ElapsedTime timer;

    public DriveTimedFollowPathAction(PathContainer container, double time) {
        this.container = container;
        this.time = time;
        this.timer = new ElapsedTime();
        Robot.getInstance().getDrive().followPath(container.buildPath());
    }

    @Override
    public void start() {

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
        return Robot.getInstance().getDrive().isDoneWithPath();
    }

    @Override
    public void done() {
        Robot.getInstance().getDrive().stop();
    }
}
