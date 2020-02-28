package com.millburnrobotics.lib.followers;

import android.util.Log;

import com.millburnrobotics.lib.control.Path;
import com.millburnrobotics.lib.geometry.Pose;
import com.millburnrobotics.lib.util.MathUtils;
import com.millburnrobotics.lib.util.PIDController;
import com.qualcomm.robotcore.util.ElapsedTime;

import static com.millburnrobotics.skystone.Constants.DriveConstants.LOOK_AHEAD;
import static com.millburnrobotics.skystone.Constants.DriveConstants.STRAFE_THRESHOLD;

public class PurePursuitFollower {
    public Path path;
    private double lastOnPath;
    private Pose currentPose = new Pose();
    private PIDController controller = new PIDController(0.033,0.025,0.15);
//    private PIDController controller = new PIDController(0.043,0.033,0.127);
    private ElapsedTime timer;

    public PurePursuitFollower(Path path) {
        this.path = path;
        lastOnPath = 0.0;
        controller.setTarget(0);
        timer = new ElapsedTime();
    }
    public Pose updatePose(Pose currentPose) {
        this.currentPose = currentPose;
        Log.d("ppursuit","l - " + path.length());
        double s = project(currentPose);
        Pose targetPose = path.get(s);
        Pose nextPose = path.get(s + LOOK_AHEAD);
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
    public double updatePower() {
        Log.d("Desmos", "(" + timer.milliseconds()/100.0 + "," + currentPose.distTo(path.end()) + ")");
        double power = -controller.getPIDOutput(currentPose.distTo(path.end()));
        Log.d("purepursuitpower", power+"");
        power = Math.abs(power);
//        if (power < 0) {
//            power = 0;
//        }
        return (power > 1 ? 1 : power);
    }
}
