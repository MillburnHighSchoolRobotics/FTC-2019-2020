package com.millburnrobotics.skystone.subsystems;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.millburnrobotics.lib.geometry.Pose;
import com.millburnrobotics.lib.util.MathUtils;
import com.millburnrobotics.skystone.Constants;
import com.millburnrobotics.skystone.Robot;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import static com.millburnrobotics.skystone.Constants.DriveConstants.BOT_WIDTH;

public class Odometry extends Subsystem {
    private String TAG = "Odometry";
    private Pose pose;
    private double x, y, heading;
    private double v_x, v_y, v_angular;
    private double orientation, rotation;
    private double erPosLast, elPosLast, ebPosLast;
    private ElapsedTime updateTimer = new ElapsedTime();

    private ElapsedTime displayTimer = new ElapsedTime();


    @Override
    public void init(boolean auto) {
        pose = new Pose(0,0,0);
        this.x = 0;
        this.y = 0;
        this.heading = 0;
        this.v_x = 0;
        this.v_y = 0;
        this.v_angular = 0;
        this.orientation = 0;
        this.rotation = 0;
        this.erPosLast = 0;
        this.elPosLast = 0;
        this.ebPosLast = 0;
        updateTimer.reset();

    }

    @Override
    public void outputToTelemetry(Telemetry telemetry, TelemetryPacket packet) {
        telemetry.addData("Pose", pose);
        telemetry.addData("er", Robot.getInstance().er.getCurrentPosition());
        telemetry.addData("el", Robot.getInstance().el.getCurrentPosition());
        telemetry.addData("eb", Robot.getInstance().eb.getCurrentPosition());

        if (displayTimer.milliseconds() > 100) {
            Log.d("OdometryDisplay", ""+pose);
            displayTimer.reset();
        }
        packet.put("x", x);
        packet.put("y", y);
        packet.put("heading", heading);

        packet.fieldOverlay().setStroke("#3F51B5");
        packet.fieldOverlay().strokeCircle(pose.getX(), pose.getY(), BOT_WIDTH/2.0);
        Pose v = pose.polar(BOT_WIDTH/2.0);
        double x1 = pose.getX() + v.getX() / 2, y1 = pose.getY() + v.getY() / 2;
        double x2 = pose.getX() + v.getX(), y2 = pose.getY() + v.getY();
        packet.fieldOverlay().strokeLine(x1, y1, x2, y2);
    }

    @Override
    public void update() {
        this.pose = getPoseEstimate();
        Robot.getInstance().savePreference("x", String.valueOf(pose.x));
        Robot.getInstance().savePreference("y", String.valueOf(pose.y));
        Robot.getInstance().savePreference("heading", String.valueOf(Math.toDegrees(pose.heading)));
    }
    private Pose getPoseEstimate() {
        double prevX = x;
        double prevY = y;
        double prevHeading = heading;
        double time = 0;

        double erPos = encoderToDistance(Robot.getInstance().er.getCurrentPosition());
        double elPos = -encoderToDistance(Robot.getInstance().el.getCurrentPosition());
        double ebPos = encoderToDistance(Robot.getInstance().eb.getCurrentPosition());
        time = updateTimer.seconds();
        updateTimer.reset();

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

        v_x = (x-prevX)/time;
        v_y = (y-prevY)/time;
        v_angular = (heading-prevHeading)/time;
        updateTimer.reset();

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
    public Pose getVelocity() {
        return new Pose(v_x, v_y, v_angular);
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
    public double getVelocityX() {
        return v_x;
    }
    public double getVelocityY() {
        return v_y;
    }
    public double getAngularVelocity() {
        return v_angular;
    }

    public static double encoderToDistance(double ticks) {
        double circumference = Math.PI* Constants.OdometryConstants.DEAD_WHEEL_DIAMETER;
        double circumferenceGeared = circumference* Constants.OdometryConstants.DEAD_WHEEL_GEARING;
        double distance = circumferenceGeared * (ticks/ Constants.OdometryConstants.DEAD_WHEEL_TICKS_PER_REV);
        return distance;
    }
}
