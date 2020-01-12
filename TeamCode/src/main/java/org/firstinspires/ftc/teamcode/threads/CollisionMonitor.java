package org.firstinspires.ftc.teamcode.threads;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.teamcode.robot.GlobalConstants;

public class CollisionMonitor extends MonitorThread {
    private static final String TAG = "CollisionMonitor";
    private final BNO055IMU imu;
    private Acceleration lastAccel;

    private boolean collision = false;


    public CollisionMonitor(Thread thread, HardwareMap hardwareMap) {
        super(thread, hardwareMap, TAG);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = false;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = null;

        synchronized (this.hardwareMap) {
            this.imu = hardwareMap.get(BNO055IMU.class, GlobalConstants.IMU_TAG);
        }
        synchronized (this.imu) {
            this.imu.initialize(parameters);
        }

        lastAccel = imu.getLinearAcceleration();

        setValue("collision",collision);
    }
    @Override
    protected void loop() {
        collision = checkCollision();
        setValue("collision", collision);
    }
    protected boolean checkCollision() {
        Acceleration accel = imu.getLinearAcceleration();
        double linear_accel_x = accel.xAccel;
        double currentJerkX = linear_accel_x - lastAccel.xAccel;
        lastAccel.xAccel = linear_accel_x;

        double linear_accel_y = accel.yAccel;
        double currentJerkY = linear_accel_y - lastAccel.yAccel;
        lastAccel.yAccel = linear_accel_y;

        lastAccel = accel;

        return !((Math.abs(currentJerkX) > GlobalConstants.COLLISION_THRESHOLD_DELTA_G) ||
                (Math.abs(currentJerkY) > GlobalConstants.COLLISION_THRESHOLD_DELTA_G));
    }
}