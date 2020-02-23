package com.millburnrobotics.skystone.subsystems;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;

import static com.millburnrobotics.skystone.Constants.IMUConstants.COLLISION_JERK_THRESHOLD;

public class IMU extends Subsystem {
    private boolean collided = false;
    private boolean collisionDetection;
    private Acceleration lastAcceleration;

    @Override
    public void init(boolean auto) {
        collisionDetection = auto;
        lastAcceleration = new Acceleration();
    }

    @Override
    public void outputToTelemetry(Telemetry telemetry) {
        telemetry.addData("Last Acceleration", lastAcceleration.xAccel + "\t" + lastAcceleration.yAccel);
    }

    @Override
    public void update() {
        Acceleration newAccel = Robot.getInstance().bno055IMU.getLinearAcceleration();
        double linear_accel_x = newAccel.xAccel;
        double currentJerkX = linear_accel_x - lastAcceleration.xAccel;

        double linear_accel_y = newAccel.yAccel;
        double currentJerkY = linear_accel_y - lastAcceleration.yAccel;

        collided = !collided && (Math.abs(currentJerkX) > COLLISION_JERK_THRESHOLD || Math.abs(currentJerkY) > COLLISION_JERK_THRESHOLD);

        lastAcceleration = newAccel;
    }
    public boolean isDetectingCollisions() {
        return collisionDetection;
    }
    public boolean collided() {
        return collided;
    }
}
