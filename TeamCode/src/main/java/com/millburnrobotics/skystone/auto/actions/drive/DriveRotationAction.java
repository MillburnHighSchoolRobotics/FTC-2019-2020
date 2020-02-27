package com.millburnrobotics.skystone.auto.actions.drive;

import com.millburnrobotics.lib.util.MathUtils;
import com.millburnrobotics.skystone.auto.actions.Action;
import com.millburnrobotics.skystone.Robot;

import static com.millburnrobotics.skystone.Constants.DriveConstants.ROTATION_THRESHOLD;

public class DriveRotationAction implements Action {

    private double targetHeading;
    private double power;
    private double threshold;

    public DriveRotationAction(double targetHeading, double power) {
        this.targetHeading = targetHeading;
        this.power = power;
        this.threshold = ROTATION_THRESHOLD;
    }
    public DriveRotationAction(double targetHeading, double power, double threshold) {
        this.targetHeading = targetHeading;
        this.power = power;
        this.threshold = threshold;
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
        return MathUtils.equals(targetHeading,Math.toDegrees(Robot.getInstance().getOdometry().getPose().heading),threshold);
    }

    @Override
    public void done() {
        Robot.getInstance().getDrive().stop();
    }
}
