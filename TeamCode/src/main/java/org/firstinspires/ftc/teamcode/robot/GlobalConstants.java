package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.qualcomm.hardware.motors.NeveRest20Gearmotor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

public class GlobalConstants {
    private static final MotorConfigurationType MOTOR_CONFIG = MotorConfigurationType.getMotorType(NeveRest20Gearmotor.class);

    public static final double WHEEL_RADIUS = 2;
    public static final double DRIVE_GEARING = 1;
    public static final double ROBOT_WIDTH = 13.7; // lf to rf
    public static final double ROBOT_LENGTH = 13.5; // lf to lbs

    public static final double INTAKE_IN_POWER = -0.6;
    public static final double INTAKE_OUT_POWER = 0.8;
    public static final double CHAINBAR_POWER = 0.6;

    public static final double CLAW_CLOSE_POS = 1;
    public static final double CLAW_OPEN_POS = 0.45;
    public static final double CLAW_NORMAL_POS = 0.5;
    public static final double CLAW_ROTATE_POS = 0;
    public static final double RIGHT_HOOK_DOWN_POS = 0.3;
    public static final double RIGHT_HOOK_UP_POS = 0.7;
    public static final double LEFT_HOOK_DOWN_POS = 0.7;
    public static final double LEFT_HOOK_UP_POS = 0.3;

    public static final int CHAINBAR_IN_TICKS = 0;
    public static final int CHAINBAR_UP_TICKS = 500;
    public static final int CHAINBAR_OUT_TICKS = 1200;


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

    public static double rpmToVelocity(double rpm) {
        return rpm * DRIVE_GEARING * 2 * Math.PI * WHEEL_RADIUS / 60.0;
    }

    public static double getMaxRpm() {
        return MOTOR_CONFIG.getMaxRPM();
    }
}
