package com.millburnrobotics.skystone.auto.modes.delivery;

import com.millburnrobotics.lib.geometry.Pose;
import com.millburnrobotics.skystone.Constants;
import com.millburnrobotics.skystone.auto.AutoModeBase;
import com.millburnrobotics.skystone.auto.actions.WaitAction;
import com.millburnrobotics.skystone.auto.actions.chainbar.ChainBarInAction;
import com.millburnrobotics.skystone.auto.actions.chainbar.ChainBarOutAction;
import com.millburnrobotics.skystone.auto.actions.drive.DriveFollowPathAction;
import com.millburnrobotics.skystone.auto.actions.drive.DriveFollowPathArmDownAction;
import com.millburnrobotics.skystone.auto.actions.drive.DriveRotationAction;
import com.millburnrobotics.skystone.auto.actions.drive.DriveTimedToPoseAction;
import com.millburnrobotics.skystone.auto.actions.drive.DriveToPoseAction;
import com.millburnrobotics.skystone.auto.actions.hook.HookDownAction;
import com.millburnrobotics.skystone.auto.actions.hook.HookUpAction;
import com.millburnrobotics.skystone.auto.actions.intake.IntakeInAction;
import com.millburnrobotics.skystone.auto.actions.sideclaw.SideClawArmDownAction;
import com.millburnrobotics.skystone.auto.actions.sideclaw.SideClawArmUpAction;
import com.millburnrobotics.skystone.auto.actions.sideclaw.SideClawCloseAction;
import com.millburnrobotics.skystone.auto.actions.sideclaw.SideClawOpenAction;
import com.millburnrobotics.skystone.paths.CyclePath;
import com.millburnrobotics.skystone.paths.DeliverPath;
import com.millburnrobotics.skystone.paths.DetectPath;
import com.millburnrobotics.skystone.paths.IntakeCyclePath;
import com.millburnrobotics.skystone.paths.ParkWallPath;
import com.millburnrobotics.skystone.subsystems.Robot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.util.Arrays;

@Autonomous(group = "Red")
public class DeliveryRedMode extends AutoModeBase {

    @Override
    public void routine() {
        Robot.getInstance().side = Constants.Side.RED;
        Robot.getInstance().getCamera().detectBlock();
        Robot.getInstance().getOdometry().setPose(new Pose(72-Constants.DriveConstants.BOT_WIDTH/2.0,-48+Constants.DriveConstants.BOT_LENGTH/2.0,Math.PI));

        // ----------------------------------- ONE BLOCK ----------------------------------- //
        runAction(new SideClawArmDownAction());
        runAction(new DriveFollowPathAction(new DetectPath(), 1, 5));
        runAction(new DriveTimedToPoseAction(new Pose(0, Robot.getInstance().getOdometry().getPose().y, Math.PI),0.6,200));

        runAction(new SideClawCloseAction());
        runAction(new WaitAction(500));

        parallelActions(Arrays.asList(
                new DriveTimedToPoseAction(new Pose(72, Robot.getInstance().getOdometry().getPose().y, Math.PI),0.4,200),
                new SideClawArmUpAction()
        ));

        runAction(new DriveToPoseAction(new Pose(-42, 24, Math.PI/2), 0.8));

        runAction(new SideClawArmDownAction());
        runAction(new WaitAction(100));
        runAction(new SideClawOpenAction());
        runAction(new SideClawArmUpAction());
        runAction(new DriveRotationAction(Math.PI, 0.7));

        // ----------------------------------- TWO BLOCK ----------------------------------- //

        if (Robot.getInstance().block == Constants.Block.RIGHT) {
            runAction(new IntakeInAction());
            runAction(new DriveFollowPathAction(new IntakeCyclePath()));
            runAction(new DriveTimedToPoseAction(new Pose(Robot.getInstance().getOdometry().getPose().x, 72,Math.PI),0.6,500));

            Robot.getInstance().getChainBar().chainBarIn();
            runAction(new WaitAction(200));
            Robot.getInstance().getChainBar().clawClose();

            runAction(new DriveToPoseAction(new Pose(-42, 24, Math.PI), 0.8));

            runAction(new ChainBarOutAction());
            runAction(new WaitAction(1000));

            Robot.getInstance().getChainBar().clawOpen();
            runAction(new WaitAction(300));

            runAction(new ChainBarInAction());
            runAction(new WaitAction(500));

        } else {
            runAction(new DriveFollowPathArmDownAction(new CyclePath(), 0));
            runAction(new DriveTimedToPoseAction(new Pose(0, Robot.getInstance().getOdometry().getPose().y, Math.PI),0.6,200));

            runAction(new SideClawCloseAction());
            runAction(new WaitAction(500));

            parallelActions(Arrays.asList(
                    new DriveTimedToPoseAction(new Pose(72, Robot.getInstance().getOdometry().getPose().y, Math.PI),0.6,200),
                    new SideClawArmUpAction()
            ));
            runAction(new DriveFollowPathAction(new DeliverPath(2), 4, 10));

            runAction(new DriveTimedToPoseAction(new Pose(0, Robot.getInstance().getOdometry().getPose().y, Math.PI),0.8,500));
            runAction(new SideClawArmDownAction());
            runAction(new WaitAction(100));
            runAction(new SideClawOpenAction());
            runAction(new SideClawArmUpAction());

            runAction(new DriveRotationAction(270, 0.8));
            runAction(new HookDownAction());
            runAction(new DriveTimedToPoseAction(new Pose(0, Robot.getInstance().getOdometry().getPose().y, Math.PI),0.8,500));
            runAction(new WaitAction(1000));
        }

        // ----------------------------------- THREE BLOCK ----------------------------------- //

//        if (Robot.getInstance().block == Constants.Block.LEFT) {
//            runAction(new DriveFollowPathArmDownAction(new CyclePath(2), 0));
//        } else {
//            runAction(new DriveFollowPathArmDownAction(new CyclePath(1), 0));
//        }
//        runAction(new DriveTimedToPoseAction(new Pose(0, Robot.getInstance().getOdometry().getPose().y, 0),0.6,400));
//
//        runAction(new SideClawCloseAction());
//        runAction(new WaitAction(500));
//
//        parallelActions(Arrays.asList(
//                new DriveTimedToPoseAction(new Pose(-72, Robot.getInstance().getOdometry().getPose().y, 0),0.6,200),
//                new SideClawArmUpAction()
//        ));
//        runAction(new DriveFollowPathAction(new DeliverPath(2), 4, 10));
//
//        runAction(new DriveTimedToPoseAction(new Pose(0, Robot.getInstance().getOdometry().getPose().y, 0),0.8,250));
//        runAction(new SideClawArmDownAction());
//        runAction(new WaitAction(100));
//        runAction(new SideClawOpenAction());
//        runAction(new SideClawArmUpAction());

        // ----------------------------------- FOUNDATION ----------------------------------- //
        runAction(new DriveTimedToPoseAction(new Pose(72, Robot.getInstance().getOdometry().getPose().y, 0),0.4,3000));
        runAction(new HookUpAction());
        runAction(new DriveFollowPathAction(new ParkWallPath()));
    }
}
