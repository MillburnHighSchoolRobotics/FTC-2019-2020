package com.millburnrobotics.lib.follower;

import android.util.Log;

import com.millburnrobotics.lib.control.Path;
import com.millburnrobotics.lib.math.MathUtils;
import com.millburnrobotics.lib.math.Pose;
import com.millburnrobotics.skystone.Constants;

public class PurePursuitFollower {
    public Path path;
    private double lookahead = Constants.DriveConstants.LOOK_AHEAD;
    private double accellerationRange = Constants.DriveConstants.ACCELERATION_RANGE;
    private double decellerationRange = Constants.DriveConstants.DECELERATION_RANGE;

    private double lastOnPath = 0.0;

    private double kp = 0.014;

    public PurePursuitFollower(Path path, double lookahead) {
        this.path = path;
        this.lookahead = lookahead;
    }
    public PurePursuitFollower(Path path) {
        this.path = path;
    }
    public Pose update(Pose currentPose) {
        double s = project(currentPose);
        Pose targetPose = path.get(s);
        Pose nextPose = path.get(s+lookahead);
        if (MathUtils.equals(s,path.length())) {
            nextPose = targetPose;
        }

        Log.d("pure pursuit","S - " + s);
        Log.d("pure pursuit","targetVector - " + targetPose.toString());
        Log.d("pure pursuit","nextVector - " + nextPose.toString());
        return nextPose;
    }
    private double project(Pose currentPos) {
        double s = path.length();

        while (true) {
            Log.d("pure pursuit","project s - " + s);
            Pose pathPos = path.get(s);
            Log.d("pure pursuit","path s pos - " + pathPos.toString());
            Pose derivPos = path.deriv(s);
            Pose dPos = currentPos.minus(pathPos);
            double ds = dPos.dot(derivPos);

            Log.d("pure pursuit", "ds - " + ds);
            if (MathUtils.equals(ds,0.0)) {
                lastOnPath = s;
                break;
            }

            s += ds / 2.0;

            if (s <= lastOnPath || s >= path.length()) {
                s = Math.max(lastOnPath, Math.min(s, path.length()));
                break;
            }
        }
        return s;
    }
    public double strafePower(double powerLow, double powerHigh, Pose currentPos) {
        double ds = Math.max(currentPos.distTo(path.start()),lastOnPath);
        double de = Math.max(currentPos.distTo(path.end()),(path.length()-lastOnPath));
        if (ds <= accellerationRange*path.length()) {
            return MathUtils.map(ds, 0,accellerationRange*path.length(),powerLow,powerHigh);
        } else if (de <= decellerationRange*path.length()) {
            return MathUtils.map(de, 0,decellerationRange*path.length(),powerLow,powerHigh);
        } else {
            return powerHigh;
        }
    }
}
