package com.millburnrobotics.skystone.auto.actions.drive;

import com.millburnrobotics.lib.geometry.Pose;
import com.millburnrobotics.lib.util.MathUtils;
import com.millburnrobotics.skystone.auto.actions.Action;
import com.millburnrobotics.skystone.Robot;
import com.qualcomm.robotcore.util.ElapsedTime;

import static com.millburnrobotics.skystone.Constants.DriveConstants.STRAFE_THRESHOLD;
import static com.millburnrobotics.skystone.Constants.IMUConstants.COLLISION_RECOVERY_TIME;

public class DriveToPoseAction implements Action {

    private Pose target;
    private double power;

    public DriveToPoseAction(Pose pose, double power) {
        this.target = pose;
        this.power = power;
    }

    @Override
    public void start() {
    }

    @Override
    public void update() {
        Pose current = Robot.getInstance().getOdometry().getPose();

        if (Robot.getInstance().getIMU().collided()) {
            Robot.getInstance().getDrive().vectorTo(target, current, 0.3);
            ElapsedTime collisionWait = new ElapsedTime();
            while (collisionWait.milliseconds() < COLLISION_RECOVERY_TIME);
        } else {
            Robot.getInstance().getDrive().vectorTo(current, target, power);
        }
    }

    @Override
    public boolean isFinished() {
        return MathUtils.equals(Robot.getInstance().getOdometry().getPose().distTo(target),0,STRAFE_THRESHOLD);
    }

    @Override
    public void done() {
        Robot.getInstance().getDrive().stop();
    }
}
