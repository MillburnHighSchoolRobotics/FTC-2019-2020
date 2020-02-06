package com.millburnrobotics.lib.followers;

import android.util.Log;

import com.millburnrobotics.lib.control.Path;
import com.millburnrobotics.lib.geometry.Pose;
import com.millburnrobotics.lib.math.MathUtils;
import com.millburnrobotics.skystone.Constants;

public class PurePursuitFollower {
    public Path path;
    private final double lookahead = Constants.DriveConstants.LOOK_AHEAD;
    private double lastOnPath = 0.0;
    private Pose nextPose;

    public PurePursuitFollower(Path path) {
        this.path = path;
    }
    public Pose updatePose(Pose currentPose) {
        Log.d("ppursuit","l - " + path.length());
        double s = project(currentPose);
        Pose targetPose = path.get(s);
        nextPose = path.get(s+lookahead);
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
