package org.firstinspires.ftc.teamcode.ThreadManager;

import android.nfc.Tag;
import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorBNO055IMUCalibration;

public class PositionMonitor extends MonitorThread {
    private static final String TAG = "PositionMonitor";
    private final String imuTag = "imu";
    private final BNO055IMU imu;
    private float rotation;
    private int turns;
    private DcMotorEx ex1;
    private DcMotorEx ex2;
    private DcMotorEx ey;
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
        float newRot;
        synchronized (imu) {
            newRot = imu.getAngularOrientation().firstAngle;
        }
        if (Math.abs(newRot - rotation) > 180) {
            if (newRot > 0) {
                turns--;
            } else {
                turns++;
            }
        }
        rotation = newRot;
        Log.d(TAG, "Rotation: " + turns + " turns, " + rotation + " degrees");
        setValue("rotation", rotation);
    }
}