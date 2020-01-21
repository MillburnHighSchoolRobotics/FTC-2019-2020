package com.millburnrobotics.skystone.robot;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.motors.NeveRest20Gearmotor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

public class GlobalConstants {
    public enum SIDE{
        BLUE, RED
    }

    public static SIDE side;

    public static final double BOT_WIDTH = 17.5;
    public static final double BOT_LENGTH = 18;

    public static final double TILE_WIDTH = 24.0;
    public static final double FOUNDATION_WIDTH = 18.5;
    public static final double FOUNDATION_LENGTH = 34.5;

    public static final String IMU_TAG = "imu 1";
    public static final double COLLISION_THRESHOLD_DELTA_G = 0.7;
    public static final double COLLISION_THRESHOLD_POSE = 3;
    public static final double COLLISION_RECOVERY_MOVEMENT = 3;
    public static final double COLLISION_RECOVERY_POWER = 0.4;
    public static final boolean CHECK_COLLISION = false;



    public static final double WHEEL_RADIUS = 2;
    public static final double WHEEL_BASE = 17.5;
    public static final double TRACK_WIDTH = 15.5;

    public static final double DEAD_WHEEL_DIAMETER = 60/25.4;
    public static final double DEAD_WHEEL_TICKS_PER_REV = 360*4;
    public static final double DEAD_WHEEL_GEARING = 32.0/48.0;

    public static final double DEAD_WHEEL_BASE_WIDTH = 15.5;
    public static final double DEAD_WHEEL_TURN_RADIUS = -2.75;

    public static final double DRIVE_POWER_HIGH = 1;
    public static final double DRIVE_POWER_LOW = 0.4;
    public static final double TURN_POWER = 0.6;

    public static final double INTAKE_IN_POWER = -0.4;
    public static final double INTAKE_IN_POWER_FAST = -0.6;
    public static final double INTAKE_OUT_POWER = 0.8;
    public static final double CHAINBAR_LOW_POWER = 0.2;
    public static final double CHAINBAR_HIGH_POWER = 0.65;
    public static final double CHAINBAR_EVEN_HIGHER_POWER = 0.6;
    public static final double LIFT_EXTENSION_POWER = 0.6;
    public static final double LIFT_RETRACTION_POWER = -0.4;
    public static final double LIFT_HOLD_POWER = 0.25;
    public static final double LIFT_DAMPEN_POWER = 1;

    public static final double RIGHT_HOOK_DOWN_POS = 1;
    public static final double RIGHT_HOOK_UP_POS = 0;
    public static final double LEFT_HOOK_DOWN_POS = 0;
    public static final double LEFT_HOOK_UP_POS = 1;

    public static final double CHAINBAR_MID_VOLTAGE = 0.38;
    public static final double CHAINBAR_IN_VOLTAGE = 0.05;
    public static final double CHAINBAR_UP_VOLTAGE = CHAINBAR_MID_VOLTAGE;
    public static final double CHAINBAR_OUT_VOLTAGE = 0.6;
    public static final double CHAINBAR_AUTO_LIFT_VOLTAGE = 0.2;
    public static final double CHAINBAR_MAX_VOLTAGE = 1; //hard limit 1.175

    public static final double CHAINBAR_CLAW_CLOSE_POS = 1;
    public static final double CHAINBAR_CLAW_OPEN_POS = 0.4;

    public static final double SIDE_BAR_UP_POS = 0.3;
    public static final double SIDE_BAR_MID_POS = 0.6;
    public static final double SIDE_BAR_DOWN_POS = 0.8;
    public static final double SIDE_CLAW_CLOSE_POS = 0.3;
    public static final double SIDE_CLAW_OPEN_POS = 1;
    public static final double SIDE_CLAW_IN_POS = 0;
    public static final double SIDE_CLAW_INIT_POS = 0.8;

    public static final double LIFT_MAX_POS = 1450;
    public static final double LIFT_RAISED_MIN_POS = 200;
    public static final double[] LIFT_STONE_POS = {0,450,730,980,1410};

//    public static Vector2d BLUE_BLOCK_2 = new Vector2d(-36,-63);
//    public static Vector2d BLUE_BLOCK_3 = new Vector2d(-36,-56);
//    public static Vector2d BLUE_BLOCK_4 = new Vector2d(-36,-48);
//    public static Vector2d BLUE_BLOCK_5 = new Vector2d(-36,-40);
//    public static Vector2d BLUE_BLOCK_6 = new Vector2d(-36,-32); // 24+4 +4
    public static Vector2d RED_BLOCK_2 = new Vector2d(36,-56);
    public static Vector2d RED_BLOCK_3 = new Vector2d(36,-48);
    public static Vector2d RED_BLOCK_4 = new Vector2d(36,-40);
    public static Vector2d RED_BLOCK_5 = new Vector2d(36,-32);
    public static Vector2d RED_BLOCK_6 = new Vector2d(36,-24); // 24+4 -4







    public static final double CLAW_EXTEND = 3;
    public static final double CLAW_TO_BACK = 6;

    public static double X_BLUE_BLOCK_CLAW = -24-BOT_WIDTH/2.0-CLAW_EXTEND;
    public static double Y_BLUE_BLOCK_CLAW = -24-4+(BOT_LENGTH/2.0-CLAW_TO_BACK);
    public static double X_BLUE_BLOCK_INTAKE = -24+2;
    public static double Y_BLUE_BLOCK_INTAKE = -72+8+BOT_LENGTH/2.0+4;
    public static Vector2d BLUE_BLOCK_1 = new Vector2d(X_BLUE_BLOCK_CLAW, Y_BLUE_BLOCK_CLAW);
    public static Vector2d BLUE_BLOCK_2 = new Vector2d(X_BLUE_BLOCK_CLAW, Y_BLUE_BLOCK_CLAW -8);
    public static Vector2d BLUE_BLOCK_3 = new Vector2d(X_BLUE_BLOCK_CLAW, Y_BLUE_BLOCK_CLAW -16);
    public static Vector2d BLUE_BLOCK_4 = new Vector2d(X_BLUE_BLOCK_CLAW, Y_BLUE_BLOCK_CLAW -24);
    public static Vector2d BLUE_BLOCK_5 = new Vector2d(X_BLUE_BLOCK_CLAW, Y_BLUE_BLOCK_CLAW -32);
    public static Vector2d BLUE_BLOCK_6 = new Vector2d(X_BLUE_BLOCK_INTAKE, Y_BLUE_BLOCK_INTAKE);

    public static double X_BLUE_DELIVERY = -24-BOT_WIDTH/2.0-CLAW_EXTEND;
    public static double Y_BLUE_DELIVERY = 72-4-FOUNDATION_LENGTH/2.0+(BOT_LENGTH/2.0-CLAW_TO_BACK);
    public static Vector2d BLUE_DELIVERY_1 = new Vector2d(X_BLUE_DELIVERY,Y_BLUE_DELIVERY +4);
    public static Vector2d BLUE_DELIVERY_2 = new Vector2d(X_BLUE_DELIVERY,Y_BLUE_DELIVERY -6);
    public static Vector2d BLUE_DELIVERY_3 = new Vector2d(X_BLUE_DELIVERY +CLAW_EXTEND,Y_BLUE_DELIVERY +4);
    public static Vector2d BLUE_DELIVERY_4 = new Vector2d(X_BLUE_DELIVERY +CLAW_EXTEND,Y_BLUE_DELIVERY -6);

    public static double X_BLUE_FOUNDATION = -48+BOT_LENGTH/2.0;
    public static double Y_BLUE_FOUNDATION = 72-4-FOUNDATION_LENGTH/2.0;
    public static Vector2d BLUE_FOUNDATION = new Vector2d(X_BLUE_FOUNDATION,Y_BLUE_FOUNDATION);

    public static Vector2d BLUE_BRIDGE_PARK = new Vector2d(-36,0);
    public static Vector2d BLUE_WALL_PARK = new Vector2d(-57,0);




    public static final long FPS_UPDATE_PERIOD = 10;
    public static final long LOOK_AHEAD = 10;

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
