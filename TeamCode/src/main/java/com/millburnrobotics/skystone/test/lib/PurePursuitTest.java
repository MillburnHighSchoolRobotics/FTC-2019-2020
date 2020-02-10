package com.millburnrobotics.skystone.test.lib;

import com.millburnrobotics.lib.control.Path;
import com.millburnrobotics.lib.control.PathBuilder;
import com.millburnrobotics.lib.geometry.Pose;
import com.millburnrobotics.lib.geometry.Waypoint;
import com.millburnrobotics.lib.util.MathUtils;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import static com.millburnrobotics.skystone.Constants.DriveConstants.LOOK_AHEAD;
import static com.millburnrobotics.skystone.Constants.DriveConstants.PURE_PURSUIT_THRESH;
import static java.lang.Math.signum;

public class PurePursuitTest {
    private static double pr = 0;
    public static void main(String[] args) {
        ElapsedTime timer = new ElapsedTime();
//        List<Double> times = new ArrayList<>();

        ArrayList<Waypoint> waypoints = new ArrayList<>();
        waypoints.add(new Waypoint(new Pose(0,0,0),0));
        waypoints.add(new Waypoint(new Pose(48,48,0),0));
        Path path =  PathBuilder.buildPath(waypoints, 0.1, 0.8, 0.9);

//        for (int i = 0; i < 1000; i++) {
//            double px = Math.random()*(waypoints.get(waypoints.size()-1).pose.x-waypoints.get(0).pose.x+1)+waypoints.get(0).pose.x;
//            double py = Math.random()*(waypoints.get(waypoints.size()-1).pose.y-waypoints.get(0).pose.y+1)+waypoints.get(0).pose.y;
//            Pose p = new Pose(px,py);
//
//            timer.reset();
//            Pose lookahead = getLookaheadPoint(p, path);
//            times.add(timer.milliseconds());
//        }
//
//        double mean = 0;
//        for (double t : times) mean += t/times.size();
//
//        System.out.println("Average Time: " + mean);
//        System.out.println("Max Time: " + Collections.max(times));
//        System.out.println("Min Time: " + Collections.min(times));
//        System.out.println("Projections: " + pr);

        Pose p = new Pose(17.3,18.4);
        timer.reset();
        Pose pose1 = getLookaheadPoint(p, path);
        double ppTime1 = timer.milliseconds();

        timer.reset();
        Pose pose2 = project(p,path);
        double ppTime2 = timer.milliseconds();

        System.out.println("(x-"+pose1.x+")^{2}+(y-"+pose1.y+")^{2}=10");
        System.out.println("(x-"+pose2.x+")^{2}+(y-"+pose2.y+")^{2}=6");
        System.out.println("(x-"+p.x+")^{2}+(y-"+p.y+")^{2}=25");
        for (double x = 0; x < path.length(); x+=0.5) {
            System.out.println("(" + path.get(x).x + "," + path.get(x).y + ")");
        }

        System.out.println("\nPure Pursuit Time1: " + ppTime1);
        System.out.println("\nPure Pursuit Time2: " + ppTime2);
        if (pr > 1)
            System.out.println("Projection");
    }
    private static Pose getLookaheadPoint(Pose currentPose, Path path) {
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
        return lookahead == null ? project(currentPose,path) : lookahead;
    }
    private static Pose project(Pose currentPos, Path path) {
        pr++;
        double s = path.length();
        double prev_ds = 0;
        while (true) {
            Pose pathPos = path.get(s);
            Pose derivPos = path.deriv(s);
            Pose dPos = currentPos.minus(pathPos);
            double ds = dPos.dot(derivPos);

            if (MathUtils.equals(ds,0.0, PURE_PURSUIT_THRESH)) {
                break;
            } else if (MathUtils.equals(Math.abs(ds),Math.abs(prev_ds))) {
                break;
            }

            prev_ds = ds;
            s += ds / 2.0;

            if (s >= path.length()) {
                break;
            }
        }
        return path.get(s+LOOK_AHEAD);
    }
}
