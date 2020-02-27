package com.millburnrobotics.skystone.auto.modes.foundation;

import com.millburnrobotics.lib.geometry.Pose;
import com.millburnrobotics.skystone.Constants;
import com.millburnrobotics.skystone.auto.AutoModeBase;
import com.millburnrobotics.skystone.auto.actions.WaitAction;
import com.millburnrobotics.skystone.auto.actions.drive.DriveRotationAction;
import com.millburnrobotics.skystone.auto.actions.drive.DriveToPoseAction;
import com.millburnrobotics.skystone.auto.actions.hook.HookDownAction;
import com.millburnrobotics.skystone.auto.actions.hook.HookUpAction;
import com.millburnrobotics.skystone.Robot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(group = "foundation")
public class FoundationRedMode extends AutoModeBase {

    @Override
    public void routine() {
        Robot.getInstance().side = Constants.Side.RED;
        Robot.getInstance().getOdometry().setPose(new Pose(63, 24+Constants.DriveConstants.BOT_WIDTH/2, 3*Math.PI/2));

        runAction(new DriveToPoseAction(new Pose(36,49,3*Math.PI/2), 0.5));

        runAction(new HookDownAction());
        timedAction(new DriveToPoseAction(new Pose(0, 49, 3*Math.PI/2),0.4), 1000);

        runAction(new WaitAction(2000));
        timedAction(new DriveToPoseAction(new Pose(66, 24+Constants.DriveConstants.BOT_WIDTH/2,3*Math.PI/2), 0.6), 3000);
        runAction(new DriveRotationAction(180, 0.3));

        runAction(new HookUpAction());
        runAction(new WaitAction(500));

        runAction(new DriveToPoseAction(new Pose(63, 0, 3*Math.PI/2), 0.5));

    }
}
