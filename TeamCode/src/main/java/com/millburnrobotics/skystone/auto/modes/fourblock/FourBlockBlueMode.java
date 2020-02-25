package com.millburnrobotics.skystone.auto.modes.fourblock;

import com.millburnrobotics.lib.control.PathBuilder;
import com.millburnrobotics.lib.geometry.Pose;
import com.millburnrobotics.lib.geometry.Waypoint;
import com.millburnrobotics.skystone.Constants;
import com.millburnrobotics.skystone.auto.AutoModeBase;
import com.millburnrobotics.skystone.auto.actions.WaitAction;
import com.millburnrobotics.skystone.auto.actions.drive.DriveFollowPathAction;
import com.millburnrobotics.skystone.subsystems.Robot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.util.ArrayList;

@Autonomous(group = "Blue")
public class FourBlockBlueMode extends AutoModeBase {
    @Override
    public void routine() {
        Robot.getInstance().side = Constants.Side.BLUE;
        Robot.getInstance().block = Constants.Block.CENTER;
//        Robot.getInstance().getCamera().detectBlock();

        double minPower = 0.2;
        double maxPower = 0.85;

        double claw_to_front = 2.75;
        double claw_extend = 1.5;

        double X_BLUE_BLOCK_CLAW = -24-9-claw_extend;
        double Y_BLUE_BLOCK_CLAW = -24-4+(9-claw_to_front);

        double X_BLUE_DELIVERY = -24-9-claw_extend;
        double Y_BLUE_DELIVERY = 72-4-34.5/2.0+(9-claw_to_front);

        double y1, y2, y3, y4;
        switch (Robot.getInstance().block) {
            case RIGHT:
                y1 = Y_BLUE_BLOCK_CLAW-40;
                y2 = Y_BLUE_BLOCK_CLAW-16;
                y3 = Y_BLUE_BLOCK_CLAW;
                y4 = Y_BLUE_BLOCK_CLAW-8;
                break;
            case CENTER:
                y1 = Y_BLUE_BLOCK_CLAW-32;
                y2 = Y_BLUE_BLOCK_CLAW-8;
                y3 = Y_BLUE_BLOCK_CLAW;
                y4 = Y_BLUE_BLOCK_CLAW-16;
                break;
            default:
            case LEFT:
                y1 = Y_BLUE_BLOCK_CLAW-24;
                y2 = Y_BLUE_BLOCK_CLAW;
                y3 = Y_BLUE_BLOCK_CLAW-8;
                y4 = Y_BLUE_BLOCK_CLAW-16;
        }

        Robot.getInstance().getOdometry().setPose(new Pose(-63, -39,Math.PI));
        ArrayList<Waypoint> w1 = new ArrayList<>(); // grab 1
        w1.add(new Waypoint(Robot.getInstance().getOdometry().getPose(),3*Math.PI/2));
        w1.add(new Waypoint(new Pose(-59,-39,Math.PI),3*Math.PI/2));
        w1.add(new Waypoint(new Pose(X_BLUE_BLOCK_CLAW,y1,Math.PI),3*Math.PI/2));
        runAction(new DriveFollowPathAction(PathBuilder.buildPath(w1),minPower, maxPower));

        runAction(new WaitAction(500));

        ArrayList<Waypoint> w2 = new ArrayList<>(); // deliver 1
        w2.add(new Waypoint(Robot.getInstance().getOdometry().getPose(),Math.toRadians(45)));
        w2.add(new Waypoint(new Pose(-42,y1+8,Math.PI),0));
        w2.add(new Waypoint(new Pose(-40,0,Math.PI),0));
        w2.add(new Waypoint(new Pose(X_BLUE_DELIVERY, Y_BLUE_DELIVERY+2,Math.PI),Math.toRadians(330)));
        runAction(new DriveFollowPathAction(PathBuilder.buildPath(w2),minPower, maxPower));

        runAction(new WaitAction(500));

        ArrayList<Waypoint> w3 = new ArrayList<>(); // cycle 2
        w3.add(new Waypoint(Robot.getInstance().getOdometry().getPose(),Math.toRadians(150)));
        w3.add(new Waypoint(new Pose(-40,0,Math.PI),Math.PI));
        w3.add(new Waypoint(new Pose(X_BLUE_BLOCK_CLAW,y2,Math.PI),Math.toRadians(215)));
        runAction(new DriveFollowPathAction(PathBuilder.buildPath(w3),minPower, maxPower));

        runAction(new WaitAction(500));

        ArrayList<Waypoint> w4 = new ArrayList<>(); // deliver 2
        w4.add(new Waypoint(Robot.getInstance().getOdometry().getPose(),Math.toRadians(45)));
        w4.add(new Waypoint(new Pose(-42,y2+8,Math.PI),0));
        w4.add(new Waypoint(new Pose(-40,0,Math.PI),0));
        w4.add(new Waypoint(new Pose(X_BLUE_DELIVERY, Y_BLUE_DELIVERY+2,Math.PI),Math.toRadians(330)));
        runAction(new DriveFollowPathAction(PathBuilder.buildPath(w4),minPower, maxPower));

        runAction(new WaitAction(500));

        ArrayList<Waypoint> w5 = new ArrayList<>(); // cycle 3
        w5.add(new Waypoint(Robot.getInstance().getOdometry().getPose(),Math.toRadians(150)));
        w5.add(new Waypoint(new Pose(-40,0,Math.PI),Math.PI));
        w5.add(new Waypoint(new Pose(X_BLUE_BLOCK_CLAW,y3,Math.PI),Math.toRadians(215)));
        runAction(new DriveFollowPathAction(PathBuilder.buildPath(w5),minPower, maxPower));

        runAction(new WaitAction(500));

        ArrayList<Waypoint> w6 = new ArrayList<>(); // deliver 3
        w6.add(new Waypoint(Robot.getInstance().getOdometry().getPose(),Math.toRadians(45)));
        w6.add(new Waypoint(new Pose(-43,y3+6,Math.PI),0));
        w6.add(new Waypoint(new Pose(-44,0,Math.PI),0));
        w6.add(new Waypoint(new Pose(X_BLUE_DELIVERY, Y_BLUE_DELIVERY-8,Math.PI),Math.toRadians(330)));
        runAction(new DriveFollowPathAction(PathBuilder.buildPath(w6),minPower, maxPower));

        runAction(new WaitAction(500));

        ArrayList<Waypoint> w7 = new ArrayList<>(); // cycle 4
        w7.add(new Waypoint(Robot.getInstance().getOdometry().getPose(),Math.toRadians(150)));
        w7.add(new Waypoint(new Pose(-40,0,Math.PI),Math.PI));
        w7.add(new Waypoint(new Pose(X_BLUE_BLOCK_CLAW,y4,Math.PI),Math.toRadians(215)));
        runAction(new DriveFollowPathAction(PathBuilder.buildPath(w7),minPower, maxPower));

        runAction(new WaitAction(500));

        ArrayList<Waypoint> w8 = new ArrayList<>(); // deliver 4
        w8.add(new Waypoint(Robot.getInstance().getOdometry().getPose(),Math.toRadians(45)));
        w8.add(new Waypoint(new Pose(-43,y4+6,Math.PI),0));
        w8.add(new Waypoint(new Pose(-44,0,Math.PI),0));
        w8.add(new Waypoint(new Pose(X_BLUE_DELIVERY, Y_BLUE_DELIVERY-8,Math.PI),Math.toRadians(330)));
        runAction(new DriveFollowPathAction(PathBuilder.buildPath(w8),minPower, maxPower));
    }
}
