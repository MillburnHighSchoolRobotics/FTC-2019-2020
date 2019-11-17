package org.firstinspires.ftc.teamcode.threads;

import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.Movement;

import static org.firstinspires.ftc.teamcode.robot.GlobalConstants.HEADING_OFFSET;
import static org.firstinspires.ftc.teamcode.robot.GlobalConstants.PENDING_OFFSET;
import static org.firstinspires.ftc.teamcode.robot.GlobalConstants.X_OFFSET;
import static org.firstinspires.ftc.teamcode.robot.GlobalConstants.Y_OFFSET;

public class PositionMonitor extends MonitorThread {
    private static final String TAG = "PositionMonitor";
    private DcMotorEx ex1;
    private DcMotorEx ex2;
    private DcMotorEx ey;

    double ex1PosLast = 0;
    double ex2PosLast = 0;
    double eyPosLast = 0;
    double offsetX = 15.5; // the left right distance from the x1 tracking wheel to the x2 tracking wheel
    double offsetY = 6; // the forward backward distance from the tracking center to the back tracking wheel

    double x = 0;
    double y = 0;
    double theta = 0;
    int rotation = 0;
    double orientation = 0;
    int count = 0;

    public PositionMonitor(Thread thread, HardwareMap hardwareMap) {
        super(thread, hardwareMap, TAG);
        ex1 = (DcMotorEx) hardwareMap.dcMotor.get("lb");
        ex2 = (DcMotorEx) hardwareMap.dcMotor.get("lf");
        ey = (DcMotorEx) hardwareMap.dcMotor.get("rf");
        ex1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ex1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ex2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ex2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ey.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ey.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    @Override
    protected void loop() {
        if (PENDING_OFFSET) {
            x = X_OFFSET;
            y = Y_OFFSET;
            theta = 2*Math.PI-HEADING_OFFSET;
            if (theta >= 2 * Math.PI) {
                theta -=  2 * Math.PI;
            } else if (theta < 0) {
                theta += 2 * Math.PI;
            }
            PENDING_OFFSET = false;
        }
        updatePosition();
        setValue("theta", 360-Movement.toDegrees(theta));
        setValue("orientation", Movement.toDegrees(orientation));
        setValue("x", x);
        setValue("y", y);
        setValue("rotation", rotation);
        setValue("count", count);
        count++;
    }
    protected void updatePosition() {
        double ex1Pos = Movement.encoderToDistance(ex1.getCurrentPosition());
        double ex2Pos = -Movement.encoderToDistance(ex2.getCurrentPosition());
        double eyPos = Movement.encoderToDistance(ey.getCurrentPosition());
        Log.d(TAG, "ex1 pos (inches): " + ex1Pos);
        Log.d(TAG, "ex2 pos (inches): " + ex2Pos);
        Log.d(TAG, "ey pos (inches): " + eyPos);
        double deltaEX1 = ex1Pos-ex1PosLast;
        double deltaEX2 = ex2Pos-ex2PosLast;
        double deltaEY = eyPos-eyPosLast;

        if (!((deltaEX1 == 0) && (deltaEX2 == 0) && (deltaEY == 0))) {
            double deltaTheta = (deltaEX2-deltaEX1)/offsetX; //radians
            Log.d(TAG, "change in theta (radians): " + deltaTheta);

            double deltaX = (0.5 * (deltaEX1 + deltaEX2)) + ((deltaEY - offsetY * deltaTheta) * Math.sin(deltaTheta));
            double deltaY = ((deltaEY - offsetY * deltaTheta) * Math.cos(deltaTheta));

            Log.d(TAG, "deltaX: " + deltaX);
            Log.d(TAG, "deltaY: " + deltaY);

            theta += deltaTheta;
            orientation += deltaTheta;
            if (theta >= 2 * Math.PI) {
                theta -=  2 * Math.PI;
                rotation ++;
            } else if (theta < 0) {
                theta += 2 * Math.PI;
                rotation --;
            }

            x += (deltaY * Math.sin(theta) + deltaX * Math.cos(theta));
            y += (deltaY * Math.cos(theta) - deltaX * Math.sin(theta));

            ex1PosLast = ex1Pos;
            ex2PosLast = ex2Pos;
            eyPosLast = eyPos;

            Log.d(TAG, "X " + x);
            Log.d(TAG, "Y: " + y);
            Log.d(TAG, "Theta (radians): " + theta);

            try {
                Thread.sleep(10);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }
}