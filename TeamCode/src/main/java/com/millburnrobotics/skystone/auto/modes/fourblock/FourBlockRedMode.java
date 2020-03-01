package com.millburnrobotics.skystone.auto.modes.fourblock;

import com.millburnrobotics.lib.control.PathBuilder;
import com.millburnrobotics.lib.geometry.Pose;
import com.millburnrobotics.lib.geometry.Waypoint;
import com.millburnrobotics.skystone.Constants;
import com.millburnrobotics.skystone.Robot;
import com.millburnrobotics.skystone.auto.AutoModeBase;
import com.millburnrobotics.skystone.auto.actions.WaitAction;
import com.millburnrobotics.skystone.auto.actions.drive.DriveFollowPathAction;
import com.millburnrobotics.skystone.auto.actions.drive.DriveFollowPathArmDownAction;
import com.millburnrobotics.skystone.auto.actions.drive.DriveRotationAction;
import com.millburnrobotics.skystone.auto.actions.drive.DriveToPoseAction;
import com.millburnrobotics.skystone.auto.actions.hook.HookDownAction;
import com.millburnrobotics.skystone.auto.actions.hook.HookMidAction;
import com.millburnrobotics.skystone.auto.actions.hook.HookUpAction;
import com.millburnrobotics.skystone.auto.actions.sideclaw.SideClawArmDownAction;
import com.millburnrobotics.skystone.auto.actions.sideclaw.SideClawArmMidAction;
import com.millburnrobotics.skystone.auto.actions.sideclaw.SideClawArmUpAction;
import com.millburnrobotics.skystone.auto.actions.sideclaw.SideClawCloseAction;
import com.millburnrobotics.skystone.auto.actions.sideclaw.SideClawOpenAction;
import com.millburnrobotics.skystone.subsystems.SideClaw;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.util.ArrayList;

@Autonomous(group = "fourblock")
public class FourBlockRedMode extends AutoModeBase {
    boolean fourblock = false;
    @Override
    public void routine() {
        Robot.getInstance().side = Constants.Side.RED;
        Robot.getInstance().getSideClaw().updateSide(SideClaw.SideClawSide.RIGHT);
//        Robot.getInstance().block = Constants.Block.RIGHT;
        Robot.getInstance().getCamera().detectBlock();

        double minPower = 0.2;
        double maxPower = 0.95;

        double claw_to_front = 3.5;
        double claw_extend = 0.5;

        double X_BLUE_BLOCK_CLAW = 24+9+claw_extend+2;
        double Y_BLUE_BLOCK_CLAW = -24-4+(9-claw_to_front);

        double X_BLUE_DELIVERY = 24+9+claw_extend;
        double Y_BLUE_DELIVERY = 72-4-34.5/2.0+(9-claw_to_front);

        double y1, y2, y3, y4;
        switch (Robot.getInstance().block) {
            case LEFT:
                y1 = Y_BLUE_BLOCK_CLAW-42;
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
            case RIGHT:
                y1 = Y_BLUE_BLOCK_CLAW-24;
                y2 = Y_BLUE_BLOCK_CLAW;
                y3 = Y_BLUE_BLOCK_CLAW-8;
                y4 = Y_BLUE_BLOCK_CLAW-16;
        }

        runAction(new SideClawArmMidAction());
        runAction(new SideClawOpenAction());

        Robot.getInstance().getOdometry().setPose(new Pose(63, -39,Math.PI));

        ArrayList<Waypoint> w1 = new ArrayList<>(); // grab 1
        w1.add(new Waypoint(new Pose(63, -39,Math.PI),Math.PI/2));
        w1.add(new Waypoint(new Pose(59,-39,Math.PI),Math.PI/2));
        w1.add(new Waypoint(new Pose(X_BLUE_BLOCK_CLAW,y1,Math.PI),Math.PI/2));
        runAction(new DriveFollowPathAction(PathBuilder.buildPath(w1),0.25, maxPower));

        grab();

        ArrayList<Waypoint> w2 = new ArrayList<>(); // deliver 1
        w2.add(new Waypoint(Robot.getInstance().getOdometry().getPose(),Math.toRadians(315)));
        w2.add(new Waypoint(new Pose(42,y1+8,Math.PI),0));
        w2.add(new Waypoint(new Pose(40,0,Math.PI),0));
        w2.add(new Waypoint(new Pose(X_BLUE_DELIVERY, Y_BLUE_DELIVERY+2,Math.PI),Math.toRadians(30)));
        runAction(new DriveFollowPathAction(PathBuilder.buildPath(w2),minPower, maxPower, 4, 7));
//        runAction(new DriveFollowPathStopAction(PathBuilder.buildPath(w2),w2.get(w2.size()-1).pose.y-2,minPower2,maxPower));

        drop();

        ArrayList<Waypoint> w3 = new ArrayList<>(); // cycle 2
        w3.add(new Waypoint(Robot.getInstance().getOdometry().getPose(),Math.toRadians(210)));
        w3.add(new Waypoint(new Pose(40,0,Math.PI),Math.PI));
        w3.add(new Waypoint(new Pose(X_BLUE_BLOCK_CLAW,y2,Math.PI),Math.toRadians(145)));
        runAction(new DriveFollowPathArmDownAction(PathBuilder.buildPath(w3),0, minPower, maxPower));

        grab();

        ArrayList<Waypoint> w4 = new ArrayList<>(); // deliver 2
        w4.add(new Waypoint(Robot.getInstance().getOdometry().getPose(),Math.toRadians(315)));
        w4.add(new Waypoint(new Pose(42,y2+8,Math.PI),0));
        w4.add(new Waypoint(new Pose(40,0,Math.PI),0));
        w4.add(new Waypoint(new Pose(X_BLUE_DELIVERY, Y_BLUE_DELIVERY+2,Math.PI),Math.toRadians(30)));
        runAction(new DriveFollowPathAction(PathBuilder.buildPath(w4),minPower, maxPower, 4, 7));
//        runAction(new DriveFollowPathStopAction(PathBuilder.buildPath(w4),w4.get(w4.size()-1).pose.y-2,minPower2,maxPower));

        drop();

        ArrayList<Waypoint> w5 = new ArrayList<>(); // cycle 3
        w5.add(new Waypoint(Robot.getInstance().getOdometry().getPose(),Math.toRadians(210)));
        w5.add(new Waypoint(new Pose(40,0,Math.PI),Math.PI));
        w5.add(new Waypoint(new Pose(X_BLUE_BLOCK_CLAW,y3,Math.PI),Math.toRadians(145)));
        runAction(new DriveFollowPathArmDownAction(PathBuilder.buildPath(w5),0, minPower, maxPower));

        grab();

        ArrayList<Waypoint> w6 = new ArrayList<>(); // deliver 3
        w6.add(new Waypoint(Robot.getInstance().getOdometry().getPose(),Math.toRadians(315)));
        w6.add(new Waypoint(new Pose(43,y3+6,Math.PI),0));
        w6.add(new Waypoint(new Pose(44,0,Math.PI),0));
        w6.add(new Waypoint(new Pose(X_BLUE_DELIVERY, Y_BLUE_DELIVERY-8,Math.PI),Math.toRadians(30)));
        runAction(new DriveFollowPathAction(PathBuilder.buildPath(w6),minPower, maxPower, 4, 7));
//        runAction(new DriveFollowPathStopAction(PathBuilder.buildPath(w6),w6.get(w6.size()-1).pose.y-2,minPower2,maxPower));

        drop();

        if (fourblock) {
            ArrayList<Waypoint> w7 = new ArrayList<>(); // cycle 4
            w7.add(new Waypoint(Robot.getInstance().getOdometry().getPose(),Math.toRadians(210)));
            w7.add(new Waypoint(new Pose(40,0,Math.PI),Math.PI));
            w7.add(new Waypoint(new Pose(X_BLUE_BLOCK_CLAW,y4,Math.PI),Math.toRadians(145)));
            runAction(new DriveFollowPathArmDownAction(PathBuilder.buildPath(w7),0, minPower, maxPower));

            grab();

            ArrayList<Waypoint> w8 = new ArrayList<>(); // deliver 4
            w5.add(new Waypoint(Robot.getInstance().getOdometry().getPose(),Math.toRadians(315)));
            w8.add(new Waypoint(new Pose(43,y4+6,Math.PI),0));
            w8.add(new Waypoint(new Pose(44,0,Math.PI),0));
            w8.add(new Waypoint(new Pose(X_BLUE_DELIVERY, Y_BLUE_DELIVERY-8,Math.PI),Math.toRadians(30)));
            runAction(new DriveFollowPathAction(PathBuilder.buildPath(w8),minPower, maxPower, 4, 7));

            drop();
        }

        runAction(new HookMidAction());
        timedAction(new DriveRotationAction(270, 0.8, 5),600); // foundation
        timedAction(new DriveToPoseAction(new Pose(Robot.getInstance().getOdometry().getPose().x,0, 3*Math.PI/2),0.8), 75);
        runAction(new HookDownAction());
        timedAction(new DriveToPoseAction(new Pose(0, Robot.getInstance().getOdometry().getPose().y,3*Math.PI/2),0.7), 500);
        runAction(new WaitAction(50));
        timedAction(new DriveToPoseAction(new Pose(72, Robot.getInstance().getOdometry().getPose().y,3*Math.PI/2),1),1300);
        timedAction(new DriveRotationAction(180, 1, 5),1000);
        runAction(new HookUpAction());

        ArrayList<Waypoint> w9 = new ArrayList<>(); // park
        w9.add(new Waypoint(Robot.getInstance().getOdometry().getPose(),Math.PI));
        w9.add(new Waypoint(new Pose(44, 0,Math.PI),Math.PI));
        runAction(new DriveFollowPathAction(PathBuilder.buildPath(w9),minPower, maxPower));
    }
    private void grab() {
        runAction(new SideClawArmDownAction());
        runAction(new SideClawCloseAction());
        timedAction(new DriveToPoseAction(new Pose(0, Robot.getInstance().getOdometry().getPose().y,Robot.getInstance().getOdometry().getPose().heading),0.4),300);
        runAction(new WaitAction(200));
        runAction(new SideClawArmUpAction());
        runAction(new WaitAction(100));
    }
    private void drop() {
        runAction(new SideClawArmDownAction());
        runAction(new SideClawOpenAction());
        runAction(new WaitAction(150));
        runAction(new SideClawArmUpAction());
    }
}
