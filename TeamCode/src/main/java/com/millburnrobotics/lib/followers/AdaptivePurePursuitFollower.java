package com.millburnrobotics.lib.followers;

import android.util.Log;

import com.millburnrobotics.lib.control.Path;
import com.millburnrobotics.lib.geometry.Pose;
import com.millburnrobotics.lib.util.MathUtils;

import static com.millburnrobotics.skystone.Constants.DriveConstants.LOOK_AHEAD;
import static com.millburnrobotics.skystone.Constants.DriveConstants.PURE_PURSUIT_THRESH;
import static java.lang.Math.signum;

public class AdaptivePurePursuitFollower {
    public Path path;
    private double lastOnPath;
    private double posOnPath;
    private boolean updateProject;
    private Pose nextPose;

    public AdaptivePurePursuitFollower(Path path) {
        this.path = path;
        lastOnPath = 0.0;
        posOnPath = 0.0;
        updateProject = false;
    }
    public Pose updatePose(Pose currentPose) {
        Log.d("pure pursuit len","l - " + path.length());
        updateProject = false;

        nextPose = getLookaheadPoint(currentPose);
        Log.d("pure pursuit","nextPose - " + nextPose);

        return nextPose;
    }
    private Pose getLookaheadPoint(Pose currentPose) {
        double inc = 1;
        Pose lookahead = null;
        Pose lastEnd = path.end();

        for (double i = path.length(); i > inc; i-=inc) {
            Pose end = lastEnd;
            Pose start = path.get(i-inc);
            lastEnd = start;

            Pose p1 = start.minus(currentPose);
            Pose p2 = end.minus(currentPose);

            double dx = p2.minus(p1).x;
            double dy = p2.minus(p1).y;
            double dr = Math.hypot(dx,dy);
            double D = p1.x * p2.y - p2.x * p1.y;

            double discriminant = LOOK_AHEAD * LOOK_AHEAD * dr * dr - D * D;
            if (discriminant < 0 || MathUtils.equals(p1.distTo(p2),0)) continue;

            Pose intersection1 = new Pose(
                    (D * dy + signum(dy) * dx * Math.sqrt(discriminant)) / (dr * dr),
                    (-D * dx + Math.abs(dy) * Math.sqrt(discriminant)) / (dr * dr)
            );
            Pose intersection2 = new Pose(
                    (D * dy - signum(dy) * dx * Math.sqrt(discriminant)) / (dr * dr),
                    (-D * dx - Math.abs(dy) * Math.sqrt(discriminant)) / (dr * dr)
            );
            boolean validIntersection1 = (Math.min(p1.x, p2.x) < intersection1.x && intersection1.x < Math.max(p1.x, p2.x))
                    || (Math.min(p1.y, p2.y) < intersection1.y && intersection1.y < Math.max(p1.y, p2.y));
            boolean validIntersection2 = (Math.min(p1.x, p2.x) < intersection2.x && intersection2.x < Math.max(p1.x, p2.x))
                    || (Math.min(p1.y, p2.y) < intersection2.y && intersection2.y < Math.max(p1.y, p2.y));

            if (validIntersection1) {
                lookahead = intersection1.plus(currentPose);
            }
            if (validIntersection2) {
                if (lookahead == null || Math.abs(intersection1.minus(p2).x) > Math.abs(intersection2.minus(p2).x) || Math.abs(intersection1.minus(p2).y) > Math.abs(intersection2.minus(p2).y)) {
                    lookahead = intersection2.plus(currentPose);
                }
            }
            if (lookahead != null) {
                break;
            }
        }

        if (path.size() > 0) {
            Pose end = path.end();
            if (end.distTo(currentPose) <= PURE_PURSUIT_THRESH) {
                return end;
            }
        }
        return lookahead == null ? path.get(project(currentPose)+LOOK_AHEAD) : lookahead;
    }
    private double project(Pose currentPos) {
        updateProject = true;
        double s = path.length();
        double prev_ds = 0;
        while (true) {
//            Log.d("pure pursuit","s - " + s);
            Pose pathPos = path.get(s);
//            Log.d("pure pursuit","path s pos - " + pathPos);
            Pose derivPos = path.deriv(s);
//            Log.d("pure pursuit","path deriv s pos - " + derivPos);
            Pose dPos = currentPos.minus(pathPos);
//            Log.d("pure pursuit","path d pos - " + dPos);
            double ds = dPos.dot(derivPos);

//            Log.d("pure pursuit", "ds - " + ds);
            if (MathUtils.equals(ds,0.0, PURE_PURSUIT_THRESH)) {
                lastOnPath = s;
                break;
            } else if (MathUtils.equals(Math.abs(ds),Math.abs(prev_ds))) {
                break;
            }

            prev_ds = ds;
            s += ds / 2.0;

            if (s <= lastOnPath || s >= path.length()) {
                s = Math.max(lastOnPath, Math.min(s, path.length()));
                break;
            }
        }
        posOnPath = s;
        return s;
    }
//    public double powerAtDistance(double s) {
//        return path.getPower(s);
//    }
//    public double updatePower(Pose pose) {
//        if (!updateProject) {
//            return path.getPower(project(pose));
//        } else {
//            return path.getPower(posOnPath);
//        }
//    }
    public Pose getNextPose() {
        return nextPose;
    }
}
