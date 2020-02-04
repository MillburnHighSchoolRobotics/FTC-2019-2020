package com.millburnrobotics.skystone.auto.modes.foundation;

import com.millburnrobotics.lib.geometry.Pose;
import com.millburnrobotics.skystone.Constants;
import com.millburnrobotics.skystone.auto.AutoModeBase;
import com.millburnrobotics.skystone.auto.actions.WaitAction;
import com.millburnrobotics.skystone.auto.actions.drive.DriveFollowPathAction;
import com.millburnrobotics.skystone.auto.actions.drive.DriveTimedFollowPathAction;
import com.millburnrobotics.skystone.auto.actions.drive.HookUpAction;
import com.millburnrobotics.skystone.auto.actions.hook.HookDownAction;
import com.millburnrobotics.skystone.paths.FoundationPullBackPath;
import com.millburnrobotics.skystone.paths.ParkWallPath;
import com.millburnrobotics.skystone.paths.ToFoundationPath;
import com.millburnrobotics.skystone.subsystems.Robot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.util.Arrays;

@Autonomous(group = "Blue")
public class FoundationBlueMode extends AutoModeBase {

    @Override
    public void routine() {
        Robot.getInstance().side = Constants.SIDE.BLUE;
        Robot.getInstance().getOdometry().setPose(new Pose(-63, 24+Constants.DriveConstants.BOT_WIDTH/2, Math.PI));

        runAction(new DriveFollowPathAction(new ToFoundationPath()));

        parallelActions(Arrays.asList(
                new DriveTimedFollowPathAction(new FoundationPullBackPath(),3),
                new HookDownAction()
        ));

        runAction(new HookUpAction());
        runAction(new WaitAction(500));

        runAction(new DriveFollowPathAction(new ParkWallPath()));

    }
}
