package org.firstinspires.ftc.teamcode.robot;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.Path;
import com.acmerobotics.roadrunner.path.PathSegment;

import org.firstinspires.ftc.teamcode.util.MathUtils;
import org.firstinspires.ftc.teamcode.util.PIDController;

import static org.firstinspires.ftc.teamcode.robot.MohanBot.shouldStop;

public class PurePursuitFollower {
    private Path path;
    private double lookahead = 6;
    private double lastOnPath = 0.0;
    private double[] headingInterpolants;

    private double decellerationRange = 0.4;
    private double kp = 0.014;

    public PurePursuitFollower(Path path, double lookahead, double[] headingInterpolants) {
        this.path = path;
        this.lookahead = lookahead;
        this.headingInterpolants = headingInterpolants;
    }
    public PurePursuitFollower(Path path, double[] headingInterpolants) {
        this.path = path;
        this.headingInterpolants = headingInterpolants;
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

        while (!shouldStop()) {
            Log.d("pure pursuit","project s - " + s);
            Vector2d pathPos = path.get(s).vec().rotated(Math.PI/2);
            Log.d("pure pursuit","path s pos - " + pathPos.toString());
            Vector2d derivPos = path.deriv(s).vec().rotated(Math.PI/2);
            Vector2d dPos = currentPos.minus(pathPos);
            double ds = dPos.dot(derivPos);

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
    public int getCurrentSegment(double s) {
        for (int p = 0; p < path.getSegments().size(); p++) {
            if ((s-path.getSegments().get(p).length()) < 0) {
                return p;
            }
            s -= path.getSegments().get(p).length();
        }
        return path.getSegments().size()-1;
    }
    public double headingCoefficient(Pose2d currentPose) {
        double heading = Math.toDegrees(currentPose.getHeading());
        double target = headingInterpolants[getCurrentSegment(currentPose.vec().distTo(end()))];
        double coeff = kp*(target-heading);
        if (MathUtils.equals(target, heading, 2)) {
            return 0;
        }
        return coeff;
    }
}
