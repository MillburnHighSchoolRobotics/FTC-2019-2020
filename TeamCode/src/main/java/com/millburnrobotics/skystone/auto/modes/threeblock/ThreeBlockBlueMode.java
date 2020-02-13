package com.millburnrobotics.skystone.auto.modes.threeblock;

import com.millburnrobotics.lib.geometry.Pose;
import com.millburnrobotics.skystone.Constants;
import com.millburnrobotics.skystone.auto.AutoModeBase;
import com.millburnrobotics.skystone.auto.actions.WaitAction;
import com.millburnrobotics.skystone.auto.actions.drive.DriveFollowPathAction;
import com.millburnrobotics.skystone.auto.actions.drive.DriveFollowPathArmDownAction;
import com.millburnrobotics.skystone.auto.actions.drive.DriveTimedToPoseAction;
import com.millburnrobotics.skystone.auto.actions.sideclaw.SideClawArmDownAction;
import com.millburnrobotics.skystone.auto.actions.sideclaw.SideClawArmUpAction;
import com.millburnrobotics.skystone.auto.actions.sideclaw.SideClawCloseAction;
import com.millburnrobotics.skystone.auto.actions.sideclaw.SideClawOpenAction;
import com.millburnrobotics.skystone.paths.CyclePath;
import com.millburnrobotics.skystone.paths.DeliverPath;
import com.millburnrobotics.skystone.paths.DetectPath;
import com.millburnrobotics.skystone.subsystems.Robot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.util.Arrays;

@Autonomous(group = "Blue")
public class ThreeBlockBlueMode extends AutoModeBase {

    @Override
    public void routine() {
        Robot.getInstance().side = Constants.Side.BLUE;
        Robot.getInstance().block = Constants.Block.CENTER;
        Robot.getInstance().getOdometry().setPose(new Pose(-72+Constants.DriveConstants.BOT_WIDTH/2.0,-48+Constants.DriveConstants.BOT_LENGTH/2.0,0));

        // ----------------------------------- ONE BLOCK ----------------------------------- //
        runAction(new SideClawArmDownAction());
        runAction(new DriveFollowPathAction(new DetectPath(), 1, 5));
//        parallelActions(Arrays.asList(
//                new DriveTimedToPoseAction(new Pose(0, Robot.getInstance().pose.y, 0),0.4,75),
//                new SideClawCloseAction()
//        ));
        runAction(new SideClawCloseAction());
        runAction(new WaitAction(500));
        runAction(new SideClawArmUpAction());

        parallelActions(Arrays.asList(
                new DriveTimedToPoseAction(new Pose(-72, Robot.getInstance().pose.y, 0),0.4,150),
                new SideClawArmUpAction()
        ));
        runAction(new DriveFollowPathAction(new DeliverPath(1), 4, 10));
//        runAction(new DriveRotationAction(0,0.6));
        parallelActions(Arrays.asList(
                new DriveTimedToPoseAction(new Pose(0, Robot.getInstance().pose.y, 0),0.4,300),
                new SideClawArmDownAction()
        ));
//        runAction(new SideClawArmDownAction());
//        runAction(new WaitAction(100));
        runAction(new SideClawOpenAction());
        runAction(new SideClawArmUpAction());

        // ----------------------------------- TWO BLOCK ----------------------------------- //

        runAction(new DriveFollowPathArmDownAction(new CyclePath(), 0));
//        runAction(new SideClawCloseAction());
        parallelActions(Arrays.asList(
                new DriveTimedToPoseAction(new Pose(0, Robot.getInstance().pose.y, 0),0.6,200),
                new SideClawCloseAction()
        ));
        runAction(new WaitAction(500));

        parallelActions(Arrays.asList(
                new DriveTimedToPoseAction(new Pose(-72, Robot.getInstance().pose.y, 0),0.6,200),
                new SideClawArmUpAction()
        ));
        runAction(new DriveFollowPathAction(new DeliverPath(2), 4, 10));
//        runAction(new DriveRotationAction(0,0.6));

        runAction(new DriveTimedToPoseAction(new Pose(0, Robot.getInstance().pose.y, 0),0.4,300));
        runAction(new SideClawArmDownAction());
        runAction(new WaitAction(100));
        runAction(new SideClawOpenAction());
        runAction(new SideClawArmUpAction());

        // ----------------------------------- THREE BLOCK ----------------------------------- //

        runAction(new DriveFollowPathArmDownAction(new CyclePath(1), 0));
        parallelActions(Arrays.asList(
                new DriveTimedToPoseAction(new Pose(0, Robot.getInstance().pose.y, 0),0.6,250),
                new SideClawCloseAction()
        ));
//        runAction(new SideClawCloseAction());
        runAction(new WaitAction(500));

        parallelActions(Arrays.asList(
                new DriveTimedToPoseAction(new Pose(-72, Robot.getInstance().pose.y, 0),0.6,200),
                new SideClawArmUpAction()
        ));
        runAction(new DriveFollowPathAction(new DeliverPath(2), 4, 10));

        runAction(new DriveTimedToPoseAction(new Pose(0, Robot.getInstance().pose.y, 0),0.6,200));
        runAction(new SideClawArmDownAction());
        runAction(new WaitAction(100));
        runAction(new SideClawOpenAction());
        runAction(new SideClawArmUpAction());
    }
}
