package com.millburnrobotics.skystone.test.modes;

import com.millburnrobotics.lib.control.PathBuilder;
import com.millburnrobotics.lib.geometry.Pose;
import com.millburnrobotics.lib.geometry.Waypoint;
import com.millburnrobotics.skystone.Constants;
import com.millburnrobotics.skystone.Robot;
import com.millburnrobotics.skystone.auto.AutoModeBase;
import com.millburnrobotics.skystone.auto.actions.WaitAction;
import com.millburnrobotics.skystone.auto.actions.drive.DriveFollowPathAction;
import com.millburnrobotics.skystone.auto.actions.drive.DriveRotationAction;
import com.millburnrobotics.skystone.auto.actions.drive.DriveToPoseAction;
import com.millburnrobotics.skystone.auto.actions.hook.HookDownAction;
import com.millburnrobotics.skystone.auto.actions.hook.HookMidAction;
import com.millburnrobotics.skystone.auto.actions.hook.HookUpAction;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.util.ArrayList;

@Autonomous(group = "test")
public class FoundationSectionTest extends AutoModeBase {

    @Override
    public void routine() {
        Robot.getInstance().side = Constants.Side.BLUE;

        double claw_to_front = 2.75;
        double claw_extend = 0.5;

        double X_BLUE_DELIVERY = -24-9-claw_extend;
        double Y_BLUE_DELIVERY = 72-4-34.5/2.0+(9-claw_to_front);

        Robot.getInstance().getOdometry().setPose(new Pose(-48, 24,Math.PI));
        runAction(new DriveToPoseAction(new Pose(X_BLUE_DELIVERY, Y_BLUE_DELIVERY-8,Math.PI), 0.5));

        runAction(new HookMidAction());
        timedAction(new DriveRotationAction(90, 0.8, 5),600); // foundation
        runAction(new HookDownAction());
        timedAction(new DriveToPoseAction(new Pose(0, Robot.getInstance().getOdometry().getPose().y-4,Math.PI/2),0.7), 500);
        runAction(new WaitAction(50));
        timedAction(new DriveToPoseAction(new Pose(-72, Robot.getInstance().getOdometry().getPose().y,Math.PI/2),1),800);
        timedAction(new DriveRotationAction(180, 1, 5),800);
        runAction(new HookUpAction());

        ArrayList<Waypoint> w9 = new ArrayList<>(); // park
        w9.add(new Waypoint(Robot.getInstance().getOdometry().getPose(),Math.PI));
        w9.add(new Waypoint(new Pose(-44, 0,Math.PI),Math.PI));
        runAction(new DriveFollowPathAction(PathBuilder.buildPath(w9),0.2, 0.85));
    }
}
