package org.firstinspires.ftc.teamcode.threads;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.GlobalConstants;
import org.firstinspires.ftc.teamcode.util.MathUtils;

public class PositionMonitor extends MonitorThread {
    private static final String TAG = "PositionMonitor";
    private DcMotorEx er;
    private DcMotorEx el;
    private DcMotorEx eb;

    double x,y,z,roll,pitch,yaw;

    double orientation;
    double rotation;
    double count = 0;

    double erPosLast = 0;
    double elPosLast = 0;
    double ebPosLast = 0;


    public PositionMonitor(Thread thread, HardwareMap hardwareMap, Pose2d start) {
        super(thread, hardwareMap, TAG);
        er = (DcMotorEx) hardwareMap.dcMotor.get("chainBar");
        el = (DcMotorEx) hardwareMap.dcMotor.get("intakeR");
        eb = (DcMotorEx) hardwareMap.dcMotor.get("intakeL");
        er.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        er.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        el.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        el.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        eb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        eb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        el.setDirection(DcMotorSimple.Direction.FORWARD);
        er.setDirection(DcMotorSimple.Direction.REVERSE);
        eb.setDirection(DcMotorSimple.Direction.FORWARD);

        x = start.getX();
        y = start.getY();
        z = 0;
        yaw = start.getHeading();
        orientation = yaw;
        rotation = 0;


        setValue("x",x);
        setValue("y",y);
        setValue("z",z);
        setValue("roll",roll);
        setValue("pitch",pitch);
        setValue("yaw",yaw);
        setValue("count",count);
        setValue("orientation", orientation);
        setValue("rotation", rotation);
    }
    @Override
    protected void loop() {
        updatePosition();
        setValue("x",x);
        setValue("y",y);
        setValue("z",z);
        setValue("roll",roll);
        setValue("pitch",pitch);
        setValue("yaw",yaw);
        setValue("orientation", orientation);
        setValue("rotation", rotation);
        setValue("count", count);
        count += 1;
    }
    protected void updatePosition() {
        ElapsedTime e = new ElapsedTime();

        double erPos = GlobalConstants.encoderToDistance(er.getCurrentPosition());
        double elPos = -GlobalConstants.encoderToDistance(el.getCurrentPosition());
        double ebPos = GlobalConstants.encoderToDistance(eb.getCurrentPosition());

        Log.d(TAG, "er pos (inches): " + erPos);
        Log.d(TAG, "el pos (inches): " + elPos);
        Log.d(TAG, "eb pos (inches): " + ebPos);

        double dr = erPos-erPosLast;
        double dl = elPos-elPosLast;
        double db = ebPos-ebPosLast;

        double dRoll = 0;
        double dPitch = 0;
        double dYaw = (dr-dl)/GlobalConstants.DEAD_WHEEL_BASE_WIDTH;
        double dx = db+(GlobalConstants.DEAD_WHEEL_TURN_RADIUS*dYaw);
        double dy = (dr+dl)/2.0;
        double dz = 0;

        Log.d(TAG, "dYaw: " + dYaw);
        Log.d(TAG, "dx: " + dx);
        Log.d(TAG, "dy: " + dy);

        double cy = Math.cos(dYaw*0.5);
        double sy = Math.sin(dYaw*0.5);
        double cp = Math.cos(dPitch*0.5);
        double sp = Math.sin(dPitch*0.5);
        double cr = Math.cos(dRoll*0.5);
        double sr = Math.sin(dRoll*0.5);

        double qw = cy*cp*cr+sy*sp*sr;
        double qx = cy*cp*sr-sy*sp*cr;
        double qy = sy*cp*sr+cy*sp*cr;
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
        roll = MathUtils.normalize(roll+dRoll);
        pitch = MathUtils.normalize(pitch+dPitch);
        yaw = MathUtils.normalize(yaw+dYaw);

        orientation += dYaw;
        rotation = orientation/(2*Math.PI);
        if (rotation > 0) rotation = Math.floor(rotation);
        else if (rotation < 0) rotation = Math.ceil(rotation);

        erPosLast = erPos;
        elPosLast = elPos;
        ebPosLast = ebPos;

        while (e.milliseconds() < GlobalConstants.FPS_UPDATE_PERIOD);
    }
}