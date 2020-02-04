package com.millburnrobotics.skystone.auto.modes.foundation;

import com.millburnrobotics.lib.geometry.Pose;
import com.millburnrobotics.skystone.Constants;
import com.millburnrobotics.skystone.auto.AutoModeBase;
import com.millburnrobotics.skystone.auto.actions.drive.DriveFollowPathAction;
<<<<<<< HEAD
import com.millburnrobotics.skystone.paths.ToFoundationPath;
=======
import com.millburnrobotics.skystone.auto.actions.hook.HookDownAction;
import com.millburnrobotics.skystone.paths.ToFoundation;
>>>>>>> e9e945f3df1aca2deff0f4d7d5c0be40e0899732
import com.millburnrobotics.skystone.subsystems.Robot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(group = "Blue")
public class FoundationBlueMode extends AutoModeBase {

    @Override
    public void routine() {
        Robot.getInstance().side = Constants.SIDE.BLUE;
<<<<<<< HEAD
        Robot.getInstance().getOdometry().setPose(new Pose(-63, 24+Constants.DriveConstants.BOT_WIDTH/2));
        runAction(new DriveFollowPathAction(new ToFoundationPath()));
=======
        runAction(new DriveFollowPathAction(new ToFoundation()));
        runAction(new HookDownAction());
>>>>>>> e9e945f3df1aca2deff0f4d7d5c0be40e0899732
    }
}
