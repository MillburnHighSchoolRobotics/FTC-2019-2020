package com.millburnrobotics.skystone;

import com.qualcomm.hardware.motors.NeveRest20Gearmotor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

public class Constants {
    public enum SIDE{
        BLUE, RED
    }

    public static final String VUFORIA_KEY = "AcSW/tj/////AAABmUB3byzZQksfqhymb0Tr3M92yvtrzF4HgDl0t7Z07OZ2xscXR1yyeX4GxftrShvm9T926ZCW0VglQKXuQE5+JkrABVijohk5DCkcE9CcxHy3mTs2Ui76Nz+9CQTgOsr6/AMLV+Te6uyXTs3rZwGdnHGRo0Q1yboJCQ51Ap2rgJc//ehVdkp/QetIMnfhRffac0edAHFt0i2F5++S/OH/4kdxFd5ha0lswd4nTnqU2MiJrz+OH4WQPQ8JC94dQZ6F3m/iX5mk4TCq/9xg3cTJvFccEUawf7PIsapABxKMJB6hcPikwa0XtyGB+vEb7fQAXZ80tRal2mcwKSHrDM4ZvYisD73X+sTIAqQnXgxYiL14";


    public static class FieldConstants {
        public static SIDE side;

        public static final double TILE_WIDTH = 24.0;
        public static final double FOUNDATION_WIDTH = 18.5;
        public static final double FOUNDATION_LENGTH = 34.5;
    }

    public static class DriveConstants {
        public static final String _LeftFrontMotor = "lf";
        public static final String _LeftBackMotor = "lb";
        public static final String _RightFrontMotor = "rf";
        public static final String _RightBackMotor = "rb";

        public static final String _IMU = "imu 1";

        public static final double BOT_WIDTH = 17.5;
        public static final double BOT_LENGTH = 18;

        public static final double COLLISION_THRESHOLD_DELTA_G = 0.7;
        public static final double COLLISION_RECOVERY_MOVEMENT = 3;
        public static final boolean CHECK_COLLISION = false;

        public static final double WHEEL_RADIUS = 2;
        public static final double WHEEL_BASE = 17.5;
        public static final double TRACK_WIDTH = 15.5;

        public static final double DRIVE_POWER_HIGH = 1;
        public static final double DRIVE_POWER_LOW = 0.4;
        public static final double TURN_POWER = 0.6;

        public static final long LOOK_AHEAD = 10;
        public static final double ACCELERATION_RANGE = 0.2;
        public static final double DECELERATION_RANGE = 0.4;

        public static final double MAX_VEL = 60;
        public static final double MAX_ACC = 40;

        public static double encoderToDistance(double ticks) {
            double circumference = Math.PI* OdometryConstants.DEAD_WHEEL_DIAMETER;
            double circumferenceGeared = circumference* OdometryConstants.DEAD_WHEEL_GEARING;
            double distance = circumferenceGeared * (ticks/ OdometryConstants.DEAD_WHEEL_TICKS_PER_REV);
            return distance;
        }
        public static double motorTicksToRadians(double ticks) {
            double rev = ticks/(MotorConfigurationType.getMotorType(NeveRest20Gearmotor.class).getTicksPerRev());
            double radians = 2*Math.PI*rev;
            return radians;
        }
    }

    public static class OdometryConstants {
        public static final double DEAD_WHEEL_DIAMETER = 60/25.4;
        public static final double DEAD_WHEEL_TICKS_PER_REV = 360*4;
        public static final double DEAD_WHEEL_GEARING = 32.0/48.0;
        public static final double DEAD_WHEEL_BASE_WIDTH = 15.5;
        public static final double DEAD_WHEEL_TURN_RADIUS = -2.75;
        public static final long FPS_UPDATE_PERIOD = 10;
    }

    public static class IntakeConstants {
        public static final String _IntakeLeft = "intakeL";
        public static final String _IntakeRight = "intakeR";

        public static final double INTAKE_IN_POWER = -0.4;
        public static final double INTAKE_IN_POWER_FAST = -0.6;
        public static final double INTAKE_OUT_POWER = 0.8;
    }


    public static class ChainBarConstants {
        public static final String _ChainBarLeft = "chainBarL";
        public static final String _ChainBarRight = "chainBarR";
        public static final String _ChainBarClaw = "chainBarClaw";

        public static final double CHAINBAR_LOW_POWER = 0.2;
        public static final double CHAINBAR_HIGH_POWER = 0.65;
        public static final double CHAINBAR_EVEN_HIGHER_POWER = 0.6;
        public static final double CHAINBAR_MID_VOLTAGE = 0.38;
        public static final double CHAINBAR_UP_VOLTAGE = CHAINBAR_MID_VOLTAGE;
        public static final double CHAINBAR_IN_VOLTAGE = 0.05;
        public static final double CHAINBAR_OUT_VOLTAGE = 0.6;
        public static final double CHAINBAR_AUTO_LIFT_VOLTAGE = 0.2;
        public static final double CHAINBAR_MAX_VOLTAGE = 1; //hard limit 1.175
        public static final double CHAINBAR_CLAW_CLOSE_POS = 1;
        public static final double CHAINBAR_CLAW_OPEN_POS = 0.4;
    }

    public static class LiftConstants {
        public static final String _LiftLeft = "liftL";
        public static final String _LiftRight = "liftR";

        public static final double LIFT_EXTENSION_POWER = 0.6;
        public static final double LIFT_RETRACTION_POWER = -0.4;
        public static final double LIFT_HOLD_POWER = 0.25;
        public static final double LIFT_DAMPEN_POWER = 1;

        public static final double LIFT_MAX_POS = 1450;
        public static final double LIFT_RAISED_MIN_POS = 200;
        public static final double[] LIFT_STONE_POS = {0,450,730,980,1410};
    }

    public static class SideClawConstants {
        public static final String _SideClawArm = "sideClawArm";
        public static final String _SideClawClaw = "sideClawClaw";

        public static final double SIDE_BAR_UP_POS = 0.3;
        public static final double SIDE_BAR_MID_POS = 0.6;
        public static final double SIDE_BAR_DOWN_POS = 0.8;

        public static final double SIDE_CLAW_CLOSE_POS = 0.3;
        public static final double SIDE_CLAW_OPEN_POS = 1;
        public static final double SIDE_CLAW_IN_POS = 0;
        public static final double SIDE_CLAW_INIT_POS = 0.8;

        public static final double CLAW_EXTEND = 3;
        public static final double CLAW_TO_BACK = 6;
    }

    public static class HookConstants {
        public static final String _FoundationHookLeft = "foundationHookL";
        public static final String _FoundationHookRight = "foundationHookR";

        public static final double RIGHT_HOOK_DOWN_POS = 1;
        public static final double RIGHT_HOOK_UP_POS = 0;
        public static final double LEFT_HOOK_DOWN_POS = 0;
        public static final double LEFT_HOOK_UP_POS = 1;
    }


}
