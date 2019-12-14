package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.hardware.motors.NeveRest20Gearmotor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

public class GlobalConstants {
    public enum SIDE{
        BLUE, RED;
    }

    public static SIDE side;

    public static final double TILE_WIDTH = 24.0;
    public static final double BASEPLATE_WIDTH = 34;

    public static final double WHEEL_RADIUS = 2;
    public static final double WHEEL_BASE = 17.5;
    public static final double TRACK_WIDTH = 15.5;

    public static final double BOT_WIDTH = 18;

    public static final double DEAD_WHEEL_DIAMETER = 60/25.4;
    public static final double DEAD_WHEEL_TICKS_PER_REV = 360*4;
    public static final double DEAD_WHEEL_GEARING = 32.0/48.0;

    public static final double DEAD_WHEEL_BASE_WIDTH = 15.5;
    public static final double DEAD_WHEEL_TURN_RADIUS = 2.75;


    public static final double INTAKE_IN_POWER = -0.6;
    public static final double INTAKE_OUT_POWER = 0.8;
    public static final double CHAINBAR_POWER = 0.8;

    public static final double CLAW_CLOSE_POS = 1;
    public static final double CLAW_OPEN_POS = 0.3;
    public static final double CLAW_NORMAL_POS = 0;
    public static final double CLAW_ROTATE_POS = 0.5;
    public static final double RIGHT_HOOK_DOWN_POS = 0;
    public static final double RIGHT_HOOK_UP_POS = 1;
    public static final double LEFT_HOOK_DOWN_POS = 0;
    public static final double LEFT_HOOK_UP_POS = 1;

    public static final int CHAINBAR_IN_TICKS = 0;
    public static final int CHAINBAR_UP_TICKS = 500;
    public static final int CHAINBAR_OUT_TICKS = 1300;

    public static final long FPS_UPDATE_PERIOD = 10;

    public static final String VUFORIA_KEY = "AcSW/tj/////AAABmUB3byzZQksfqhymb0Tr3M92yvtrzF4HgDl0t7Z07OZ2xscXR1yyeX4GxftrShvm9T926ZCW0VglQKXuQE5+JkrABVijohk5DCkcE9CcxHy3mTs2Ui76Nz+9CQTgOsr6/AMLV+Te6uyXTs3rZwGdnHGRo0Q1yboJCQ51Ap2rgJc//ehVdkp/QetIMnfhRffac0edAHFt0i2F5++S/OH/4kdxFd5ha0lswd4nTnqU2MiJrz+OH4WQPQ8JC94dQZ6F3m/iX5mk4TCq/9xg3cTJvFccEUawf7PIsapABxKMJB6hcPikwa0XtyGB+vEb7fQAXZ80tRal2mcwKSHrDM4ZvYisD73X+sTIAqQnXgxYiL14";

    public static double encoderToDistance(double ticks) {
        double circumference = Math.PI*DEAD_WHEEL_DIAMETER;
        double circumferenceGeared = circumference*DEAD_WHEEL_GEARING;
        double distance = circumferenceGeared * (ticks/DEAD_WHEEL_TICKS_PER_REV);
        return distance;
    }
    public static double motorTicksToRadians(double ticks) {
        double rev = ticks/(MotorConfigurationType.getMotorType(NeveRest20Gearmotor.class).getTicksPerRev());
        double radians = 2*Math.PI*rev;
        return radians;
    }
}
