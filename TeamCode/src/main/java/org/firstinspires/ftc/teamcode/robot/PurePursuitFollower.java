package org.firstinspires.ftc.teamcode.robot;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.Path;

import org.firstinspires.ftc.teamcode.util.MathUtils;

import static org.firstinspires.ftc.teamcode.robot.MohanBot.shouldStop;

public class PurePursuitFollower {
    private Path path;
    private double lookahead = 6;
    private double lastOnPath = 0.0;

    private double decellerationRange = 0.4;

    public PurePursuitFollower(Path path, double lookahead) {
        this.path = path;
        this.lookahead = lookahead;
    }
    public PurePursuitFollower(Path path) {
        this.path = path;
    }
    public Vector2d update(Pose2d currentPose) {
        double s = projectPoint(currentPose.vec());
        Vector2d targetVector = (path.get(s).vec().rotated(Math.PI/2));
        Vector2d nextVector = (path.get(s+lookahead).vec().rotated(Math.PI/2));
        if (MathUtils.equals(s,path.length())) {
            nextVector = targetVector;
        }

        Log.d("pure pursuit","S - " + s);
        Log.d("pure pursuit","targetVector - " + targetVector.toString());
        Log.d("pure pursuit","nextVector - " + nextVector.toString());
        return nextVector;
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
    public double getPower(double powerLow, double powerHigh, Vector2d currentPos) {
        double d = Math.max(currentPos.distTo(end()),(path.length()-lastOnPath));
        if (d <= decellerationRange*path.length()) {
            return MathUtils.map(d, 0,decellerationRange*path.length(),powerLow,powerHigh);
        } else {
            return powerHigh;
        }
    }
}
