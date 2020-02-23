package com.millburnrobotics.skystone.auto.modes.foundation;

import com.millburnrobotics.lib.geometry.Pose;
import com.millburnrobotics.skystone.Constants;
import com.millburnrobotics.skystone.auto.AutoModeBase;
import com.millburnrobotics.skystone.auto.actions.WaitAction;
import com.millburnrobotics.skystone.auto.actions.drive.DriveFollowPathAction;
import com.millburnrobotics.skystone.auto.actions.drive.DriveRotationAction;
import com.millburnrobotics.skystone.auto.actions.drive.DriveToPoseAction;
import com.millburnrobotics.skystone.auto.actions.hook.HookDownAction;
import com.millburnrobotics.skystone.auto.actions.hook.HookUpAction;
import com.millburnrobotics.skystone.paths.FoundationPullBackPath;
import com.millburnrobotics.skystone.paths.ParkWallPath;
import com.millburnrobotics.skystone.paths.ToFoundationPath;
import com.millburnrobotics.skystone.subsystems.Robot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(group = "Red")
public class FoundationRedMode extends AutoModeBase {

    @Override
    public void routine() {
        Robot.getInstance().side = Constants.Side.RED;
        Robot.getInstance().getOdometry().setPose(new Pose(63, 24+Constants.DriveConstants.BOT_WIDTH/2, 3*Math.PI/2));

        runAction(new DriveFollowPathAction(new ToFoundationPath()));

        runAction(new HookDownAction());
        timedAction(new DriveToPoseAction(new Pose(0, 49, 3*Math.PI/2),0.4), 1000);

        runAction(new WaitAction(2000));
        timedAction(new DriveFollowPathAction(new FoundationPullBackPath()), 3000);
        runAction(new DriveRotationAction(180, 0.3));

        runAction(new HookUpAction());
        runAction(new WaitAction(500));

        runAction(new DriveFollowPathAction(new ParkWallPath()));

    }
}
