package com.millburnrobotics.skystone.auto.actions.drive;

import com.millburnrobotics.lib.util.MathUtils;
import com.millburnrobotics.skystone.auto.actions.Action;
import com.millburnrobotics.skystone.subsystems.Robot;

import static com.millburnrobotics.skystone.Constants.DriveConstants.ROTATION_THRESHOLD;

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
        return MathUtils.equals(targetHeading,Math.toDegrees(Robot.getInstance().getOdometry().getPose().heading),ROTATION_THRESHOLD);
    }

    @Override
    public void done() {
        Robot.getInstance().getDrive().stop();
    }
}
