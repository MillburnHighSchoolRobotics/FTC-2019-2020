package com.millburnrobotics.lib.follower;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.Path;

import com.millburnrobotics.lib.math.MathUtils;
import com.millburnrobotics.skystone.robot.GlobalConstants;
import com.millburnrobotics.skystone.robot.MohanBot;

public class PurePursuitFollower {
    private Path path;
    private double lookahead = GlobalConstants.LOOK_AHEAD;
    private double lastOnPath = 0.0;
    private double[][] headingInterpolants;

    private double decellerationRange = 0.4;
    private double kp = 0.014;

    public PurePursuitFollower(Path path, double lookahead, double[] angles) {
        this.path = path;
        this.lookahead = lookahead;
        this.headingInterpolants = new double[path.getSegments().size()][2];

        int pathLength = 0;
        for (int i = 0; i < headingInterpolants.length; i++) {
            pathLength += path.getSegments().get(i).length();
            this.headingInterpolants[i] = new double[] {angles[i],pathLength};
        }
    }
    public PurePursuitFollower(Path path, double[] angles) {
        this.path = path;
        this.headingInterpolants = new double[path.getSegments().size()][2];

        int pathLength = 0;
        for (int i = 0; i < headingInterpolants.length; i++) {
            pathLength += path.getSegments().get(i).length();
            this.headingInterpolants[i] = new double[] {angles[i],pathLength};
        }
    }
    public Vector2d updatePosition(Vector2d currentPos) {
        double s = projectPoint(currentPos);
        Vector2d targetPos = path.get(s).vec().rotated(Math.PI/2);
        Vector2d nextPos = path.get(s+lookahead).vec().rotated(Math.PI/2);
        if (MathUtils.equals(s,path.length())) {
            nextPos = targetPos;
        }

        Log.d("pure pursuit","S - " + s);
        Log.d("pure pursuit","targetVector - " + targetPos.toString());
        Log.d("pure pursuit","nextVector - " + nextPos.toString());
        return nextPos;
    }
    private double projectPoint(Vector2d currentPos) {
        double s = path.length();

        while (!MohanBot.shouldStop()) {
            Log.d("pure pursuit","project s - " + s);
            Vector2d pathPos = path.get(s).vec().rotated(Math.PI/2);
            Log.d("pure pursuit","path s pos - " + pathPos.toString());
            Vector2d derivPos = path.deriv(s).vec().rotated(Math.PI/2);
            Vector2d dPos = currentPos.minus(pathPos);
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
    public Vector2d end() {
        return path.end().vec().rotated(Math.PI/2);
    }
    public double strafePower(double powerLow, double powerHigh, Vector2d currentPos) {
        double d = Math.max(currentPos.distTo(end()),(path.length()-lastOnPath));
        if (d <= decellerationRange*path.length()) {
            return MathUtils.map(d, 0,decellerationRange*path.length(),powerLow,powerHigh);
        } else {
            return powerHigh;
        }
    }
    public double headingCV(Pose2d currentPose, double rotationThreshold) {
        double heading = Math.toDegrees(currentPose.getHeading());
        double s = projectPoint(currentPose.vec());
        double target = headingInterpolants[headingInterpolants.length-1][0];
        for (int r = 0; r < headingInterpolants.length; r++) {
            if (s <= headingInterpolants[r][1]) {
                target = headingInterpolants[r][0];
                break;
            }
        }
        if (heading - target > 180) {
            heading -= 360;
        } else if (target - heading > 180) {
            heading += 360;
        }
        double u = kp*(target-heading);
        if (MathUtils.equals(target, heading, rotationThreshold)) {
            u = 0;
        }
        return u;
    }
    public Vector2d reeeCollision(Pose2d currentPose) {
        double deriv = MathUtils.normalize(Math.PI-path.deriv(projectPoint(currentPose.vec())).getHeading());
        return new Vector2d(GlobalConstants.COLLISION_RECOVERY_MOVEMENT*Math.cos(deriv),GlobalConstants.COLLISION_RECOVERY_MOVEMENT*Math.sin(deriv));
    }
}
