package com.millburnrobotics.skystone.subsystems;

import android.util.Log;

import com.millburnrobotics.lib.geometry.Pose;
import com.millburnrobotics.lib.util.MathUtils;
import com.millburnrobotics.skystone.Constants;
import com.millburnrobotics.skystone.Robot;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Odometry extends Subsystem {
    private String TAG = "Odometry";
    private Pose pose;
    private double x, y, heading;
    private double orientation, rotation;
    private double erPosLast, elPosLast, ebPosLast;

    private ElapsedTime displayTimer = new ElapsedTime();


    @Override
    public void init(boolean auto) {
        pose = new Pose(0,0,0);
        this.x = 0;
        this.y = 0;
        this.heading = 0;
        this.orientation = 0;
        this.rotation = 0;
        this.erPosLast = 0;
        this.elPosLast = 0;
        this.ebPosLast = 0;
    }

    @Override
    public void outputToTelemetry(Telemetry telemetry) {
        telemetry.addData("Pose", pose);
        telemetry.addData("er", Robot.getInstance().er.getCurrentPosition());
        telemetry.addData("el", Robot.getInstance().el.getCurrentPosition());
        telemetry.addData("eb", Robot.getInstance().eb.getCurrentPosition());

        if (displayTimer.milliseconds() > 100) {
            Log.d("OdometryDisplay", ""+pose);
            displayTimer.reset();
        }
    }

    @Override
    public void update() {
        this.pose = getPoseEstimate();
        Robot.getInstance().savePreference("x", String.valueOf(pose.x));
        Robot.getInstance().savePreference("y", String.valueOf(pose.y));
        Robot.getInstance().savePreference("heading", String.valueOf(Math.toDegrees(pose.heading)));
    }
    private Pose getPoseEstimate() {
        double erPos = encoderToDistance(Robot.getInstance().er.getCurrentPosition());
        double elPos = -encoderToDistance(Robot.getInstance().el.getCurrentPosition());
        double ebPos = encoderToDistance(Robot.getInstance().eb.getCurrentPosition());

        Log.d(TAG, "er pos (inches): " + erPos);
        Log.d(TAG, "el pos (inches): " + elPos);
        Log.d(TAG, "eb pos (inches): " + ebPos);

        double dr = erPos-erPosLast;
        double dl = elPos-elPosLast;
        double db = ebPos-ebPosLast;

        double dHeading = (dr-dl)/ Constants.OdometryConstants.DEAD_WHEEL_BASE_WIDTH;
        double dx = db+(Constants.OdometryConstants.DEAD_WHEEL_TURN_RADIUS*dHeading);
        double dy = (dr+dl)/2.0;

        Log.d(TAG, "dYaw: " + dHeading);
        Log.d(TAG, "dx: " + dx);
        Log.d(TAG, "dy: " + dy);

        heading = MathUtils.normalize(heading+dHeading);

        double s,c;
        if (MathUtils.equals(dHeading,0)) {
            s = 1-dHeading*dHeading/6.0;
            c = dHeading/2.0;
        } else {
            s = Math.sin(dHeading)/dHeading;
            c = (1-Math.cos(dHeading))/dHeading;
        }

        double dfX = dx*s-dy*c;
        double dfY = dx*c+dy*s;

        double dxp = dfX*Math.cos(heading)-dfY*Math.sin(heading);
        double dyp = dfX*Math.sin(heading)+dfY*Math.cos(heading);

        x += dxp;
        y += dyp;

        orientation += dHeading;
        rotation = orientation/(2*Math.PI);
        if (rotation > 0) rotation = Math.floor(rotation);
        else if (rotation < 0) rotation = Math.ceil(rotation);

        Log.d(TAG, "rotation: " + rotation);
        Log.d(TAG, "orientation: " + orientation);

        Log.d(TAG, "x: " + x);
        Log.d(TAG, "y: " + y);
        Log.d(TAG, "heading: " + heading);

        erPosLast = erPos;
        elPosLast = elPos;
        ebPosLast = ebPos;

        return new Pose(x,y,heading);
    }

    public void setPose(Pose pose) {
        this.pose = pose;
        this.x = pose.x;
        this.y = pose.y;
        this.heading = pose.heading;
        this.orientation = pose.heading;
        this.rotation = 0;
        this.erPosLast = 0;
        this.elPosLast = 0;
        this.ebPosLast = 0;
    }
    public Pose getPose() {
        return pose;
    }
    public double getX() {
        return x;
    }
    public double getY() {
        return y;
    }
    public double getHeading() {
        return heading;
    }

    public static double encoderToDistance(double ticks) {
        double circumference = Math.PI* Constants.OdometryConstants.DEAD_WHEEL_DIAMETER;
        double circumferenceGeared = circumference* Constants.OdometryConstants.DEAD_WHEEL_GEARING;
        double distance = circumferenceGeared * (ticks/ Constants.OdometryConstants.DEAD_WHEEL_TICKS_PER_REV);
        return distance;
    }
}
