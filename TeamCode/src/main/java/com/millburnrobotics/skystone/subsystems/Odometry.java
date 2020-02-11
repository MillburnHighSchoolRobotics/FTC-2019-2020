package com.millburnrobotics.skystone.subsystems;

import android.util.Log;

import com.millburnrobotics.lib.geometry.Pose;
import com.millburnrobotics.lib.util.MathUtils;
import com.millburnrobotics.skystone.Constants;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Odometry extends Subsystem {
    private String TAG = "Odometry";
    private Pose pose;
    private double x, y, z, yaw, pitch, roll;
    private double orientation, rotation;
    private double erPosLast, elPosLast, ebPosLast;

    @Override
    public void init(boolean auto) {
        pose = new Pose(0,0,0);
        this.x = 0;
        this.y = 0;
        this.z = 0;
        this.yaw = 0;
        this.pitch = 0;
        this.roll = 0;
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

        Log.d("OdometryDisplay", ""+pose);
    }

    @Override
    public void update() {
        this.pose = getPoseEstimate();
    }
    private Pose getPoseEstimate() {
        double erPos = Constants.DriveConstants.encoderToDistance(Robot.getInstance().er.getCurrentPosition());
        double elPos = -Constants.DriveConstants.encoderToDistance(Robot.getInstance().el.getCurrentPosition());
        double ebPos = Constants.DriveConstants.encoderToDistance(Robot.getInstance().eb.getCurrentPosition());

        Log.d(TAG, "er pos (inches): " + erPos);
        Log.d(TAG, "el pos (inches): " + elPos);
        Log.d(TAG, "eb pos (inches): " + ebPos);

        double dr = erPos-erPosLast;
        double dl = elPos-elPosLast;
        double db = ebPos-ebPosLast;

        double dRoll = 0;
        double dPitch = 0;
        double dYaw = (dr-dl)/ Constants.OdometryConstants.DEAD_WHEEL_BASE_WIDTH;
        double dx = db+(Constants.OdometryConstants.DEAD_WHEEL_TURN_RADIUS*dYaw);
        double dy = (dr+dl)/2.0;
        double dz = 0;

        Log.d(TAG, "dYaw: " + dYaw);
        Log.d(TAG, "dx: " + dx);
        Log.d(TAG, "dy: " + dy);

        roll = MathUtils.normalize(roll+dRoll);
        pitch = MathUtils.normalize(pitch+dPitch);
        yaw = MathUtils.normalize(yaw+dYaw);

        double cy = Math.cos(yaw*0.5);
        double sy = Math.sin(yaw*0.5);
        double cp = Math.cos(pitch*0.5);
        double sp = Math.sin(pitch*0.5);
        double cr = Math.cos(roll*0.5);
        double sr = Math.sin(roll*0.5);

        double qw = cy*cp*cr+sy*sp*sr;
        double qx = cy*cp*sr-sy*sp*cr;
        double qy = cy*sp*cr+sy*cp*sr;
        double qz = sy*cp*cr-cy*sp*sr;

        Log.d(TAG, "q: (" + qw + "," + qx + "," + qy + "," + qz +")");

        double x1 = 1-2*(qy*qy+qz*qz);
        double y1 = 2*(qx*qy-qz*qw);
        double z1 = 2*(qx*qz+qy*qw);
        double x2 = 2*(qx*qy+qz*qw);
        double y2 = 1-2*(qx*qx+qz*qz);
        double z2 = 2*(qy*qz-qx*qw);
        double x3 = 2*(qx*qz-qy*qw);
        double y3 = 2*(qy*qz+qx*qw);
        double z3 = 1-2*(qx*qx+qy*qy);

        double rx = x1*dx+y1*dy+z1*dz;
        double ry = x2*dx+y2*dy+z2*dz;
        double rz = x3*dx+y3*dy+z3*dz;

        x += rx;
        y += ry;
        z += rz;

        orientation += dYaw;
        rotation = orientation/(2*Math.PI);
        if (rotation > 0) rotation = Math.floor(rotation);
        else if (rotation < 0) rotation = Math.ceil(rotation);

        Log.d(TAG, "rotation: " + rotation);
        Log.d(TAG, "orientation: " + orientation);

        Log.d(TAG, "x: " + x);
        Log.d(TAG, "y: " + y);
        Log.d(TAG, "z: " + z);
        Log.d(TAG, "yaw: " + yaw);
        Log.d(TAG, "pitch: " + pitch);
        Log.d(TAG, "roll: " + roll);

        erPosLast = erPos;
        elPosLast = elPos;
        ebPosLast = ebPos;

        return new Pose(x,y,yaw);
    }

    public void setPose(Pose pose) {
        this.pose = pose;
        this.x = pose.x;
        this.y = pose.y;
        this.z = 0;
        this.yaw = pose.heading;
        this.pitch = 0;
        this.roll = 0;
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
        return yaw;
    }
}
