package com.millburnrobotics.skystone.auto.actions.drive;

import android.util.Log;

import com.millburnrobotics.lib.control.Path;
import com.millburnrobotics.lib.geometry.Pose;
import com.millburnrobotics.skystone.Robot;
import com.millburnrobotics.skystone.auto.actions.Action;

import static com.millburnrobotics.skystone.Constants.DriveConstants.STRAFE_THRESHOLD;

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
        Log.d("followpathaction", "here");
        return Robot.getInstance().getDrive().getLookahead().equals(path.end());
    }

    @Override
    public void done() {
//        while (Robot.getInstance().getOdometry().getPose().distTo(path.end()) > STRAFE_THRESHOLD) {
//            Robot.getInstance().getDrive().vectorTo(Robot.getInstance().getOdometry().getPose(), path.end(), Robot.getInstance().getDrive().getFFpower());
//        }
        Robot.getInstance().getDrive().stop();
    }
}
