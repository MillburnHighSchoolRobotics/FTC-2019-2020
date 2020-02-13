package com.millburnrobotics.lib.followers;

import android.util.Log;

import com.millburnrobotics.lib.control.Path;
import com.millburnrobotics.lib.geometry.Pose;
import com.millburnrobotics.lib.util.MathUtils;

import static com.millburnrobotics.skystone.Constants.DriveConstants.LOOK_AHEAD;
import static com.millburnrobotics.skystone.Constants.DriveConstants.PURE_PURSUIT_THRESH;
import static java.lang.Math.signum;

public class PurePursuitFollower {
    public Path path;
    private double lastOnPath;
    private Pose nextPose;

    public PurePursuitFollower(Path path) {
        this.path = path;
        lastOnPath = 0.0;
    }
    public Pose updatePose(Pose currentPose) {
        Log.d("ppursuit","l - " + path.length());
        double s = project(currentPose);
        Pose targetPose = path.get(s);
        nextPose = path.get(s+LOOK_AHEAD);
        if (MathUtils.equals(s,path.length())) {
            nextPose = targetPose;
        }

        Log.d("pure pursuit","S - " + s);
        Log.d("pure pursuit","targetVector - " + targetPose.toString());
        Log.d("pure pursuit","nextVector - " + nextPose.toString());

        return nextPose;
    }
    private double project(Pose currentPos) {
        double s = Math.floor(path.length()*1000.0)/1000.0;

        while (true) {
            Log.d("pure pursuit","s - " + s);
            Pose pathPos = path.get(s);
            Log.d("pure pursuit","path s pos - " + pathPos);
            Pose derivPos = path.deriv(s);
            Log.d("pure pursuit","path deriv s pos - " + derivPos);
            Pose dPos = currentPos.minus(pathPos);
            Log.d("pure pursuit","path d pos - " + dPos);
            double ds = dPos.dot(derivPos);

            Log.d("pure pursuit", "ds - " + ds);
            if (MathUtils.equals(ds,0.0, 0.25)) {
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
    public double powerAtDistance(double s) {
        return path.getPower(s);
    }
    public double updatePower() {
        return path.getPower(lastOnPath);
    }
    public double getLastOnPath() {
        return lastOnPath;
    }
    public Pose getNextPose() {
        return nextPose;
    }
}
