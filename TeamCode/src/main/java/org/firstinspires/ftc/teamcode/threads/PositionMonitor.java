package org.firstinspires.ftc.teamcode.threads;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robot.GlobalConstants;
import org.firstinspires.ftc.teamcode.util.MathUtils;


public class PositionMonitor extends MonitorThread {
    private static final String TAG = "PositionMonitor";
    private DcMotorEx er;
    private DcMotorEx el;
    private DcMotorEx eb;

    double x;
    double y;
    double theta;
    double orientation;
    double rotation;
    double count = 0;

    double erPosLast = 0;
    double elPosLast = 0;
    double ebPosLast = 0;


    public PositionMonitor(Thread thread, HardwareMap hardwareMap, Pose2d start) {
        super(thread, hardwareMap, TAG);
        er = (DcMotorEx) hardwareMap.dcMotor.get("er");
        el = (DcMotorEx) hardwareMap.dcMotor.get("el");
        eb = (DcMotorEx) hardwareMap.dcMotor.get("eb");
        er.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        er.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        el.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        el.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        eb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        eb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        x = start.getX();
        y = start.getY();
        theta = start.getHeading();
        orientation = theta;
        rotation = 0;


        setValue("x",start.getX());
        setValue("y",start.getY());
        setValue("theta",start.getHeading());
    }
    @Override
    protected void loop() {
        updatePosition();
        setValue("x", x);
        setValue("y", y);
        setValue("theta", theta);
        setValue("orientation", orientation);
        setValue("rotation", rotation);
        setValue("count", count);
        count += 1;
    }
    protected void updatePosition() {
        double erPos = GlobalConstants.encoderToDistance(er.getCurrentPosition());
        double elPos = -GlobalConstants.encoderToDistance(el.getCurrentPosition());
        double ebPos = GlobalConstants.encoderToDistance(eb.getCurrentPosition());

        Log.d(TAG, "er pos (inches): " + erPos);
        Log.d(TAG, "el pos (inches): " + elPos);
        Log.d(TAG, "eb pos (inches): " + ebPos);

        double dr = erPos-erPosLast;
        double dl = elPos-elPosLast;
        double db = ebPos-ebPosLast;

        double dTheta = (dr-dl)/GlobalConstants.DEAD_WHEEL_BASE_WIDTH;
        double dX = db+(GlobalConstants.DEAD_WHEEL_TURN_RADIUS*dTheta);
        double dY = (dr+dl)/2.0;

        Log.d(TAG, "dTheta: " + dTheta);
        Log.d(TAG, "dX: " + dX);
        Log.d(TAG, "dY: " + dY);

        double s,c;
        if (MathUtils.equals(dTheta,0)) {
            s = 1-dTheta*dTheta/6.0;
            c = dTheta/2.0;
        } else {
            s = Math.sin(dTheta)/dTheta;
            c = (1-Math.cos(dTheta))/dTheta;
        }

        double dfX = dX*s-dY*c;
        double dfY = dX*c+dY*s;

        double dXPose = dfX*Math.cos(theta)-dfY*Math.sin(theta);
        double dYPose = dfX*Math.sin(theta)+dfY*Math.cos(theta);

        x += dXPose;
        y += dYPose;
        orientation += dTheta;
        theta = MathUtils.normalize(theta+dTheta);
        rotation = orientation/(2*Math.PI);
        if (rotation > 0) rotation = Math.floor(rotation);
        else if (rotation < 0) rotation = Math.ceil(rotation);

        erPosLast = erPos;
        elPosLast = elPos;
        ebPosLast = ebPos;

        try {
            Thread.sleep(GlobalConstants.FPS_UPDATE_PERIOD);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }
}