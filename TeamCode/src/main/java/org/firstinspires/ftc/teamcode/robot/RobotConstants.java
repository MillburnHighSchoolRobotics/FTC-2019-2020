package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.qualcomm.hardware.motors.NeveRest20Gearmotor;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

public class RobotConstants {
    private static final MotorConfigurationType MOTOR_CONFIG =
            MotorConfigurationType.getMotorType(NeveRest20Gearmotor.class);
    private static final double TICKS_PER_REV = MOTOR_CONFIG.getTicksPerRev();

    public static double WHEEL_RADIUS = 2;
    public static double DRIVE_GEARING = 1;
    public static double ROBOT_WIDTH = 13.7; // lf to rf
    public static double ROBOT_LENGTH = 13.5; // lf to lb

    public static double X_OFFSET, Y_OFFSET, HEADING_OFFSET = 0;

    public static double kV = 0.013955236664657396;// / rpmToVelocity(getMaxRpm());
    public static double kA = 0.000045225265421946806;
    public static double kStatic = 0.05955177567008853;

    public static PIDCoefficients TRANSLATION_PID = new PIDCoefficients(1, 0, 0);
    public static PIDCoefficients HEADING_PID = new PIDCoefficients(0, 0, 0);

    public static DriveConstraints BASE_CONSTRAINTS = new DriveConstraints(
            50.0, 40.0, 0.0,
            Math.toRadians(180), Math.toRadians(180), 0.0
    );

    public static double encoderTicksToInches(int ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * DRIVE_GEARING * ticks / MOTOR_CONFIG.getTicksPerRev();
    }

    public static double rpmToVelocity(double rpm) {
        return rpm * DRIVE_GEARING * 2 * Math.PI * WHEEL_RADIUS / 60.0;
    }

    public static double getMaxRpm() {
        return MOTOR_CONFIG.getMaxRPM();
    }
}
