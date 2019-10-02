package org.firstinspires.ftc.teamcode.drive.localizer;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.threads.ThreadManager;

/*
 * Sample tracking wheel localizer implementation assuming the standard configuration:
 *
 *    /--------------\
 *    |     ____     |
 *    |     ----     |
 *    | ||        || |
 *    | ||        || |
 *    |              |
 *    |              |
 *    \--------------/
 *
 * Note: this could be optimized significantly with REV bulk reads
 */

public class MohanLocalizer implements Localizer {
    public static double TICKS_PER_REV = 360*4;
    public static double WHEEL_RADIUS = (60/25.4)/2; // in
    public static double GEAR_RATIO = 32/48.0; // output (wheel) speed / input (encoder) speed

//    DcMotorEx ex1;
//    DcMotorEx ex2;
//    DcMotorEx ey;
    static Pose2d mohansLocation = new Pose2d(0,0,0);
    double count = 0;

    double xOffset = 0;
    double yOffset = 0;
    double thetaOffset = 0;

    public MohanLocalizer(HardwareMap hardwareMap) {
//        ex1 = (DcMotorEx) hardwareMap.dcMotor.get("lf");
//        ex2 = (DcMotorEx) hardwareMap.dcMotor.get("rf");
//        ey = (DcMotorEx) hardwareMap.dcMotor.get("lb");
    }

    public static double encoderTicksToInches(int ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    @Override
    public Pose2d getPoseEstimate() {
        return mohansLocation;
    }

    @Override
    public void setPoseEstimate(Pose2d pose2d) {
        this.xOffset = pose2d.getX();
        this.yOffset = pose2d.getY();
        this.thetaOffset = pose2d.getHeading();
        mohansLocation = pose2d;
    }

    @Override
    public void update() {
//        double x, y, theta;
//        do {
//            x = ThreadManager.getInstance().getValue("x", Double.class);
//            y = ThreadManager.getInstance().getValue("y", Double.class);
//            theta = Math.toRadians(ThreadManager.getInstance().getValue("theta", Double.class));
//        } while (mohansLocation.getX() == x && mohansLocation.getY() == y && mohansLocation.getHeading() == theta);

        double count1;
        do {
            count1 = ThreadManager.getInstance().getValue("count", Integer.class);
        } while (count == count1);


        double x = ThreadManager.getInstance().getValue("x", Double.class) + xOffset;
        double y = ThreadManager.getInstance().getValue("y", Double.class) + yOffset;
        double theta = Angle.norm(Math.toRadians(ThreadManager.getInstance().getValue("theta", Double.class)) + thetaOffset);
        mohansLocation = new Pose2d(x,y,theta);

        count = count1;
    }
}
