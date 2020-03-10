package com.millburnrobotics.lib.followers;

import com.millburnrobotics.lib.control.Path;
import com.millburnrobotics.lib.geometry.Pose;
import com.millburnrobotics.lib.util.MathUtils;

import static com.millburnrobotics.skystone.Constants.DriveConstants.LOOK_AHEAD;
import static java.lang.Math.signum;

public class AdaptivePurePursuitFollower {
    private Path path;
    private Pose last_lookahead = new Pose();
    private int check = 0;
    private double lastOnPath;
    public double distAlongPath;

    public AdaptivePurePursuitFollower(Path path) {
        lastOnPath = 0.0;
        this.path = path;
    }
    public void update(Pose currentPose) {
        this.distAlongPath = project(currentPose);
    }
    public Pose getLookaheadPoint(Pose currentPose) {
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

        if (lookahead == null) {
            lookahead = updatePose();
        }
        if (path.size() > 0) {
            Pose end = path.end();
            if (end.equals(lookahead)) {
                last_lookahead = end;
                return end;
            }
        }
        if (check > 1) {
            last_lookahead = lookahead;
            check = 0;
        }
        check++;
//        Log.d("pplookahead", ""+lookahead);
        return lookahead;
    }
    public Pose updatePose() {
        Pose targetPose = path.get(distAlongPath);
        Pose nextPose = path.get(distAlongPath + LOOK_AHEAD);
        if (MathUtils.equals(distAlongPath,path.length())) {
            nextPose = targetPose;
        }
        return nextPose;
    }
    public double project(Pose currentPos) {
        double s = Math.floor(path.length()*1000.0)/1000.0;

        while (true) {
            Pose pathPos = path.get(s);
            Pose derivPos = path.deriv(s);
            Pose dPos = currentPos.minus(pathPos);
            double ds = dPos.dot(derivPos);

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
}
