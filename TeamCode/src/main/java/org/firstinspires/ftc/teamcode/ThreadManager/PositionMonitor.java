package org.firstinspires.ftc.teamcode.ThreadManager;

import android.util.Log;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.RobotControl.Movement;

public class PositionMonitor extends MonitorThread {
    private static final String TAG = "PositionMonitor";
    private final String imuTag = "imu";
    private final BNO055IMU imu;
    private float rotation;
    private int turns;
    private DcMotorEx ex1;
    private DcMotorEx ex2;
    private DcMotorEx ey;

    double ex1PosLast = 0;
    double ex2PosLast = 0;
    double eyPosLast = 0;
    double ex1PosLastReset = 0;
    double ex2PosLastReset = 0;
    double thetaLastReset = 0;
    double offsetX1 = 7.75; // the left right distance form the tracking center ot the x1 tracking wheel
    double offsetX2 = 7.75; // the left right distance from the tracking center to the x2 tracking wheel
    double offsetY = 5; // the forward backward distance from the tracking center to the back tracking wheel
    double lastTheta = 0;
    double encoderTolerance = 5;

    double x = 0;
    double y = 0;
    double theta = 0;

    public PositionMonitor(Thread thread, HardwareMap hardwareMap) {
        super(thread, hardwareMap, TAG);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = false;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = null;
        synchronized (this.hardwareMap) {
            this.imu = hardwareMap.get(BNO055IMU.class, this.imuTag);
        }
        synchronized (this.imu) {
            this.imu.initialize(parameters);
        }
        turns = 0;
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
//        float newRot;
//        synchronized (imu) {
//            newRot = imu.getAngularOrientation().firstAngle;
//        }
//        if (Math.abs(newRot - rotation) > 180) {
//            if (newRot > 0) {
//                turns--;
//            } else {
//                turns++;
//            }
//        }
//        rotation = newRot;
//
//
//
//        Log.d(TAG, "Rotation: " + turns + " turns, " + rotation + " degrees");
//        setValue("rotation", rotation);
        updatePosition();
        setValue("theta", theta);
        setValue("x", x);
        setValue("y", y);
    }
    protected void updatePosition() {
        double ex1Pos = Movement.getDistanceTravelled(ex1.getCurrentPosition());
        double ex2Pos = -Movement.getDistanceTravelled(ex2.getCurrentPosition());
        double eyPos = Movement.getDistanceTravelled(ey.getCurrentPosition());
        double deltaX1 = ex1Pos-ex1PosLast;
        double deltaX2 = ex2Pos-ex2PosLast;
        double deltaY = eyPos-eyPosLast;
        ex1PosLast = ex1Pos;
        ex2PosLast = ex2Pos;
        eyPosLast = eyPos;

        if (!((deltaX1 == 0) && (deltaX2 == 0) && (deltaY == 0))) {
            double deltaResetX1 = ex1Pos-ex1PosLastReset;
            double deltaResetX2 = ex2Pos-ex2PosLastReset;

            double currentTheta = thetaLastReset + (deltaResetX1-deltaResetX2)/(offsetX1+offsetX2); //radians
            double deltaTheta = currentTheta - lastTheta;
            lastTheta = currentTheta;

            double localOffsetX;
            double localOffsetY;
            if (Math.abs(deltaTheta) < encoderTolerance) {
                localOffsetX = deltaY;
                localOffsetY = deltaX2;
            } else {
                localOffsetX = 2 * Math.sin(deltaTheta/2) * ((deltaY/deltaTheta) + offsetY);
                localOffsetY = 2 * Math.sin(deltaTheta/2) * ((deltaX2/deltaTheta) + offsetX2);
            }

            double localOffsetR = Math.sqrt(Math.pow(localOffsetX,2)+Math.pow(localOffsetY,2));
            double localOffsetTheta = 0;
            if ((localOffsetY >= 0) && (localOffsetR != 0)) {
                localOffsetTheta = Math.acos(localOffsetX/localOffsetR);
            } else if (localOffsetR != 0) {
                localOffsetTheta = -Math.acos(localOffsetX/localOffsetR);
            }

            localOffsetTheta -= (lastTheta + (deltaTheta/2));

            localOffsetX = localOffsetR * Math.cos(localOffsetTheta);
            localOffsetY = localOffsetR * Math.sin(localOffsetTheta);

            x += localOffsetX;
            y += localOffsetY;
            theta = currentTheta*(180/Math.PI);
        }
    }
}