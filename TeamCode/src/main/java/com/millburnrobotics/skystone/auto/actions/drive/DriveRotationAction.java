package com.millburnrobotics.skystone.auto.actions.drive;

import com.millburnrobotics.lib.geometry.Pose;
import com.millburnrobotics.skystone.auto.actions.Action;
import com.millburnrobotics.skystone.paths.PathContainer;
import com.millburnrobotics.skystone.subsystems.Robot;

public class DriveRotationAction implements Action {

    private PathContainer container;
    private double targetHeading;
    private double power;

    public DriveRotationAction(double targetHeading, double power) {
        this.targetHeading = targetHeading;
        this.power = power;
    }

    @Override
    public void start() {

    }

    @Override
    public void update() {
        Pose current = Robot.getInstance().getOdometry().getPose();
        Robot.getInstance().getDrive().rotateTo(targetHeading, power);
    }

    @Override
    public boolean isFinished() {
        return Robot.getInstance().getDrive().isDoneRotating();
    }

    @Override
    public void done() {
        Robot.getInstance().getDrive().stop();
    }
}
