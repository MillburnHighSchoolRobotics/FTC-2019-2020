package com.millburnrobotics.skystone.auto.autons;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.millburnrobotics.skystone.Constants;

public class AutonConstants {
    public static Vector2d RED_BLOCK_2 = new Vector2d(36,-56);
    public static Vector2d RED_BLOCK_3 = new Vector2d(36,-48);
    public static Vector2d RED_BLOCK_4 = new Vector2d(36,-40);
    public static Vector2d RED_BLOCK_5 = new Vector2d(36,-32);
    public static Vector2d RED_BLOCK_6 = new Vector2d(36,-24); // 24+4 -4

    public static double X_BLUE_BLOCK_CLAW = -24- Constants.DriveConstants.BOT_WIDTH/2.0- Constants.SideClawConstants.CLAW_EXTEND;
    public static double Y_BLUE_BLOCK_CLAW = -24-4+(Constants.DriveConstants.BOT_LENGTH/2.0- Constants.SideClawConstants.CLAW_TO_BACK);

    public static Vector2d BLUE_BLOCK_5 = new Vector2d(X_BLUE_BLOCK_CLAW, Y_BLUE_BLOCK_CLAW -32);
    public static Vector2d BLUE_BLOCK_4 = new Vector2d(X_BLUE_BLOCK_CLAW, Y_BLUE_BLOCK_CLAW -24);
    public static Vector2d BLUE_BLOCK_3 = new Vector2d(X_BLUE_BLOCK_CLAW, Y_BLUE_BLOCK_CLAW -16);
    public static Vector2d BLUE_BLOCK_2 = new Vector2d(X_BLUE_BLOCK_CLAW, Y_BLUE_BLOCK_CLAW -8);
    public static Vector2d BLUE_BLOCK_1 = new Vector2d(X_BLUE_BLOCK_CLAW, Y_BLUE_BLOCK_CLAW);

    public static double X_BLUE_BLOCK_INTAKE = -24+2;
    public static double Y_BLUE_BLOCK_INTAKE = -72+8+ Constants.DriveConstants.BOT_LENGTH/2.0+4;

    public static Vector2d BLUE_BLOCK_6 = new Vector2d(X_BLUE_BLOCK_INTAKE, Y_BLUE_BLOCK_INTAKE);

    public static double X_BLUE_DELIVERY = -24- Constants.DriveConstants.BOT_WIDTH/2.0- Constants.SideClawConstants.CLAW_EXTEND;
    public static double Y_BLUE_DELIVERY = 72-4- Constants.FieldConstants.FOUNDATION_LENGTH/2.0+(Constants.DriveConstants.BOT_LENGTH/2.0- Constants.SideClawConstants.CLAW_TO_BACK);

    public static Vector2d BLUE_DELIVERY_4 = new Vector2d(X_BLUE_DELIVERY + Constants.SideClawConstants.CLAW_EXTEND,Y_BLUE_DELIVERY -6);
    public static Vector2d BLUE_DELIVERY_3 = new Vector2d(X_BLUE_DELIVERY + Constants.SideClawConstants.CLAW_EXTEND,Y_BLUE_DELIVERY +4);
    public static Vector2d BLUE_DELIVERY_2 = new Vector2d(X_BLUE_DELIVERY,Y_BLUE_DELIVERY -6);
    public static Vector2d BLUE_DELIVERY_1 = new Vector2d(X_BLUE_DELIVERY,Y_BLUE_DELIVERY +4);

    public static double X_BLUE_FOUNDATION = -48+ Constants.DriveConstants.BOT_LENGTH/2.0;
    public static double Y_BLUE_FOUNDATION = 72-4- Constants.FieldConstants.FOUNDATION_LENGTH/2.0;
    public static Vector2d BLUE_FOUNDATION = new Vector2d(X_BLUE_FOUNDATION,Y_BLUE_FOUNDATION);

    public static Vector2d BLUE_BRIDGE_PARK = new Vector2d(-36,0);
    public static Vector2d BLUE_WALL_PARK = new Vector2d(-57,0);
}
