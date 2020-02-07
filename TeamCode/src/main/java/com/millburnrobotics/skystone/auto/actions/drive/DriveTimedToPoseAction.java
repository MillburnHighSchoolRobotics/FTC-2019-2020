package com.millburnrobotics.skystone.auto.actions.drive;

import com.millburnrobotics.lib.geometry.Pose;
import com.millburnrobotics.lib.util.MathUtils;
import com.millburnrobotics.skystone.auto.actions.Action;
import com.millburnrobotics.skystone.subsystems.Robot;
import com.qualcomm.robotcore.util.ElapsedTime;

import static com.millburnrobotics.skystone.Constants.DriveConstants.STRAFE_THRESHOLD;

public class DriveTimedToPoseAction implements Action {

    private Pose target;
    private double power;
    private double time;
    private ElapsedTime timer;

    public DriveTimedToPoseAction(Pose pose, double power, double time) {
        this.target = pose;
        this.power = power;
        this.time = time;
    }

    @Override
    public void start() {
        this.timer = new ElapsedTime();
    }

    @Override
    public void update() {
        Pose current = Robot.getInstance().getOdometry().getPose();
        Robot.getInstance().getDrive().vectorTo(current, target, power);
    }

    @Override
    public boolean isFinished() {
        if (this.timer.milliseconds() >= time) {
            return true;
        }
        return MathUtils.equals(Robot.getInstance().getOdometry().getPose().distTo(target),0,STRAFE_THRESHOLD);
    }

    @Override
    public void done() {
        Robot.getInstance().getDrive().stop();
    }
}
