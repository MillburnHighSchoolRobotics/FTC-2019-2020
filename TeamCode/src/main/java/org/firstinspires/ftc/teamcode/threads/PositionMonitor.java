package org.firstinspires.ftc.teamcode.threads;

import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.autonomous.Movement;

public class PositionMonitor extends MonitorThread {
    private static final String TAG = "PositionMonitor";
//    private final String imuTag = "imu";
//    private final BNO055IMU imu;
    private DcMotorEx ex1;
    private DcMotorEx ex2;
    private DcMotorEx ey;

    double ex1PosLast = 0;
    double ex2PosLast = 0;
    double eyPosLast = 0;
    double offsetX = 15.53; // the left right distance from the x1 tracking wheel to the x1 tracking wheel
    double offsetY = 5; // the forward backward distance from the tracking center to the back tracking wheel

    double x = 0;
    double y = 0;
    double theta = 0;
    int rotation = 0;
    double orientation = 0;

    public PositionMonitor(Thread thread, HardwareMap hardwareMap) {
        super(thread, hardwareMap, TAG);
//        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
//        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
//        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
//        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
//        parameters.loggingEnabled = false;
//        parameters.loggingTag = "IMU";
//        parameters.accelerationIntegrationAlgorithm = null;
//        synchronized (this.hardwareMap) {
//            this.imu = hardwareMap.get(BNO055IMU.class, this.imuTag);
//        }
//        synchronized (this.imu) {
//            this.imu.initialize(parameters);
//        }
        ex1 = (DcMotorEx) hardwareMap.dcMotor.get("lf");
        ex2 = (DcMotorEx) hardwareMap.dcMotor.get("lb");
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
//        float newTheta;
//        synchronized (imu) {
//            newTheta = imu.getAngularOrientation().firstAngle;
//        }
//        if (Math.abs(newTheta - theta) > 180) {
//            if (newTheta > 0) {
//                rotation--;
//            } else {
//                rotation++;
//            }
//        }
//        theta = newTheta;
//
//        Log.d(TAG, "Rotation: " + rotation + " theta, " + theta + " degrees");
//        setValue("theta", theta);

        updatePosition();
        setValue("theta", Movement.toDegrees(theta));
        setValue("orientation", Movement.toDegrees(orientation));
        setValue("x", x); // reverse x and y
        setValue("y", y);
        setValue("rotation", rotation);
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
            double deltaTheta = (deltaEX1-deltaEX2)/offsetX; //radians
            Log.d(TAG, "change in theta (radians): " + deltaTheta);

            double deltaX = (0.5 * (deltaEX1 + deltaEX2)) + ((deltaEY - offsetY * deltaTheta) * Math.sin(deltaTheta));
            double deltaY = ((deltaEY - offsetY * deltaTheta) * Math.cos(deltaTheta));

            Log.d(TAG, "deltaX: " + deltaX);
            Log.d(TAG, "deltaY: " + deltaY);

            theta += deltaTheta;
            orientation = theta;
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
                Thread.sleep(5);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

//            double localOffsetX;
//            double localOffsetY;
//            if (Math.abs(deltaTheta) < encoderTolerance) {
//                localOffsetX = deltaY;
//                localOffsetY = deltaX2;
//            } else {
//                localOffsetX = 2 * Math.sin(deltaTheta/2) * ((deltaY/deltaTheta) + offsetY);
//                localOffsetY = 2 * Math.sin(deltaTheta/2) * ((deltaX2/deltaTheta) + offsetX2);
//            }
//
//            double localOffsetR = Math.sqrt(Math.pow(localOffsetX,2)+Math.pow(localOffsetY,2));
//            double localOffsetTheta = 0;
//            if ((localOffsetY >= 0) && (localOffsetR != 0)) {
//                localOffsetTheta = Math.acos(localOffsetX/localOffsetR);
//            } else if (localOffsetR != 0) {
//                localOffsetTheta = -Math.acos(localOffsetX/localOffsetR);
//            }
//
//            localOffsetTheta -= (lastTheta + (deltaTheta/2));
//
//            localOffsetX = localOffsetR * Math.cos(localOffsetTheta);
//            localOffsetY = localOffsetR * Math.sin(localOffsetTheta);
//
//            x += localOffsetX;
//            y += localOffsetY;
//            theta = currentTheta*(180/Math.PI);
        }
    }
}