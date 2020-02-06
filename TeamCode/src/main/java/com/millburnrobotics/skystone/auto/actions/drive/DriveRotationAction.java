package com.millburnrobotics.skystone.auto.actions.drive;

import com.millburnrobotics.lib.math.MathUtils;
import com.millburnrobotics.skystone.auto.actions.Action;
import com.millburnrobotics.skystone.subsystems.Robot;

public class DriveRotationAction implements Action {

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
        Robot.getInstance().getDrive().rotateTo(targetHeading, power);
    }

    @Override
    public boolean isFinished() {
        return MathUtils.equals(targetHeading,Math.toDegrees(Robot.getInstance().pose.heading),Robot.getInstance().getDrive().rotationThreshold);
    }

    @Override
    public void done() {
        Robot.getInstance().getDrive().stop();
    }
}
