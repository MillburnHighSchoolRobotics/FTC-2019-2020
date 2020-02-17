package com.millburnrobotics.skystone;

import com.millburnrobotics.lib.geometry.Pose;
import com.millburnrobotics.lib.util.MathUtils;
import com.millburnrobotics.skystone.subsystems.Drive;
import com.millburnrobotics.skystone.subsystems.Hook;
import com.millburnrobotics.skystone.subsystems.Intake;
import com.millburnrobotics.skystone.subsystems.Lift;
import com.millburnrobotics.skystone.subsystems.Robot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import static com.millburnrobotics.skystone.Constants.ChainBarConstants.CHAINBARL_IN_POS;
import static com.millburnrobotics.skystone.Constants.ChainBarConstants.CHAINBARL_OUT_POS;
import static com.millburnrobotics.skystone.Constants.ChainBarConstants.CHAINBAR_INCREMENT;
import static com.millburnrobotics.skystone.Constants.DriveConstants.DRIVE_POWER_HIGH;
import static com.millburnrobotics.skystone.Constants.DriveConstants.DRIVE_POWER_LOW;
import static com.millburnrobotics.skystone.Constants.LiftConstants.LIFT_RAISED_MIN_POS;
import static com.millburnrobotics.skystone.Constants.LiftConstants.LIFT_STONE_POS;
import static com.millburnrobotics.skystone.Constants.SideClawConstants.SIDE_ARM_DOWN_POS;
import static com.millburnrobotics.skystone.Constants.SideClawConstants.SIDE_ARM_INCREMENT;
import static com.millburnrobotics.skystone.Constants.SideClawConstants.SIDE_ARM_UP_POS;
import static com.millburnrobotics.skystone.Constants.SideClawConstants.SIDE_CLAW_INCREMENT;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(group = "teleop")
public class TeleOp extends OpMode {
    private ElapsedTime odomUpdate = new ElapsedTime();

    @Override
    public void init() {
        Robot.getInstance().side = Constants.Side.BLUE;
        Robot.getInstance().init(hardwareMap, false);
        double pref_x = Double.valueOf(Robot.getInstance().readPreference("x"));
        double pref_y = Double.valueOf(Robot.getInstance().readPreference("y"));
        double pref_heading = Double.valueOf(Robot.getInstance().readPreference("heading"));
        Robot.getInstance().getOdometry().setPose(new Pose(pref_x, pref_y, pref_heading));
    }

    @Override
    public void loop() {
        //-------------------------------------------- Intake --------------------------------------------//
        if (MathUtils.equals(gamepad1.left_trigger, 1, 0.05)) {
            Robot.getInstance().getIntake().intakeOut();
        } else if (MathUtils.equals(gamepad1.right_trigger, 1, 0.05)) {
            Robot.getInstance().getIntake().intakeIn();
        } else {
            Robot.getInstance().getIntake().stop();
        }

        //-------------------------------------------- Hook --------------------------------------------//
        if (gamepad1.x && Robot.getInstance().getHook().toggleHook()){ // toggle foundation hook
            if (Robot.getInstance().getHook().getState() == Hook.HookState.HOOK_DOWN) {
                Robot.getInstance().getHook().hookUp();
            } else {
                Robot.getInstance().getHook().hookDown();
            }
        }

        //-------------------------------------------- Side Claw --------------------------------------------//
        if (MathUtils.equals(gamepad2.left_trigger, 1, 0.05) && Robot.getInstance().getSideClaw().getArmPosition() > SIDE_ARM_DOWN_POS && Robot.getInstance().getSideClaw().canToggleSideArm()) {
            Robot.getInstance().getSideClaw().setArmPosition(Robot.getInstance().getSideClaw().getArmPosition() - SIDE_ARM_INCREMENT);
        } else if (MathUtils.equals(gamepad2.right_trigger, 1, 0.05) && Robot.getInstance().getSideClaw().getArmPosition() < SIDE_ARM_UP_POS && Robot.getInstance().getSideClaw().canToggleSideArm()) {
            Robot.getInstance().getSideClaw().setArmPosition(Robot.getInstance().getSideClaw().getArmPosition() + SIDE_ARM_INCREMENT);
        }
        if (gamepad2.left_bumper && Robot.getInstance().getSideClaw().getClawPosition() < 1 && Robot.getInstance().getSideClaw().canToggleSideClaw()) {
            Robot.getInstance().getSideClaw().setClawPosition(Robot.getInstance().getSideClaw().getClawPosition() + SIDE_CLAW_INCREMENT);
        } else if (gamepad2.right_bumper && Robot.getInstance().getSideClaw().getClawPosition() > 0 && Robot.getInstance().getSideClaw().canToggleSideClaw()) {
            Robot.getInstance().getSideClaw().setClawPosition(Robot.getInstance().getSideClaw().getClawPosition() - SIDE_CLAW_INCREMENT);
        }
        //-------------------------------------------- ChainBar --------------------------------------------//
        if (gamepad1.right_bumper && Robot.getInstance().getChainBar().getChainBarLPosition() < CHAINBARL_OUT_POS && Robot.getInstance().getChainBar().canToggleChainBar()) {
            Robot.getInstance().getChainBar().setChainBarPosition(
                    Robot.getInstance().getChainBar().getChainBarLPosition() + CHAINBAR_INCREMENT,
                    Robot.getInstance().getChainBar().getChainBarRPosition() - CHAINBAR_INCREMENT
            );
        } else if (gamepad1.left_bumper && Robot.getInstance().getChainBar().getChainBarLPosition() > CHAINBARL_IN_POS && Robot.getInstance().getChainBar().canToggleChainBar()) {
            Robot.getInstance().getChainBar().setChainBarPosition(
                    Robot.getInstance().getChainBar().getChainBarLPosition() - CHAINBAR_INCREMENT,
                    Robot.getInstance().getChainBar().getChainBarRPosition() + CHAINBAR_INCREMENT
            );
        } else if (gamepad1.y || Robot.getInstance().getIntake().getState() == Intake.IntakeState.INTAKE_IN) {
            Robot.getInstance().getChainBar().chainBarUp();
        } else if (gamepad1.dpad_right) {
           Robot.getInstance().getChainBar().chainBarInAuto();
        } else if (Robot.getInstance().getChainBar().isChainBarIn()) {
           Robot.getInstance().getChainBar().chainBarInUpdate();
        }

        //-------------------------------------------- Claw --------------------------------------------//
        if (gamepad1.a && Robot.getInstance().getChainBar().canToggleClaw()) {
            Robot.getInstance().getChainBar().clawClose();
        } else if (gamepad1.b && Robot.getInstance().getChainBar().canToggleClaw()) {
            Robot.getInstance().getChainBar().clawOpen();
        } else if (gamepad1.y || Robot.getInstance().getIntake().getState() == Intake.IntakeState.INTAKE_IN) {
            Robot.getInstance().getChainBar().clawClose();
        } else if (gamepad1.dpad_right) {
            Robot.getInstance().getChainBar().clawClose();
        }

        //-------------------------------------------- Lift --------------------------------------------//
        if (gamepad2.dpad_up && Robot.getInstance().getLift().getLiftTargetBlock() < LIFT_STONE_POS.length){
            Robot.getInstance().getLift().updateLiftTargetBlock(Robot.getInstance().getLift().getLiftTargetBlock() + 1);
        } else if (gamepad2.dpad_down && Robot.getInstance().getLift().getLiftTargetBlock() > 1) {
            Robot.getInstance().getLift().updateLiftTargetBlock(Robot.getInstance().getLift().getLiftTargetBlock() - 1);
        }
        if (gamepad2.a) {
            if (Robot.getInstance().getLift().getState() == Lift.LiftState.FAIL) {
                Robot.getInstance().getLift().setState(Lift.LiftState.WORK);
            } else {
                Robot.getInstance().getLift().setState(Lift.LiftState.FAIL);
            }
        }
        if (gamepad1.dpad_up) {
            Robot.getInstance().getLift().manualLiftUp();
        } else if (gamepad1.dpad_down) {
            Robot.getInstance().getLift().manualLiftDown();
        } else if (gamepad1.y) {
            Robot.getInstance().getLift().autoLiftReset();
        } else if (Robot.getInstance().getLift().isLiftUpReset()) {
            Robot.getInstance().getLift().autoLiftDown();
        } else if (Robot.getInstance().getLift().getState() != Lift.LiftState.FAIL){
            if (gamepad1.dpad_left) {
                Robot.getInstance().getLift().autoLiftToBlock();
            }
            Robot.getInstance().getLift().updateLiftPID();
        } else {
            Robot.getInstance().getLift().setLiftPower(0);
        }

        //-------------------------------------------- Movement --------------------------------------------
        if (gamepad2.y) {
            if (Robot.getInstance().getDrive().getState() == Drive.DriveState.FIELD_CENTRIC) {
                Robot.getInstance().getDrive().setState(Drive.DriveState.ROBOT_CENTRIC);
            } else {
                Robot.getInstance().getDrive().setState(Drive.DriveState.FIELD_CENTRIC);
            }
        }
        if (gamepad2.x) {
            Robot.getInstance().getOdometry().setPose(new Pose());
        }
        double drivePower = (Robot.getInstance().liftR.getCurrentPosition() > LIFT_RAISED_MIN_POS) ? DRIVE_POWER_LOW : DRIVE_POWER_HIGH;
        double strafeX = (Robot.getInstance().getDrive().getState() == Drive.DriveState.FIELD_CENTRIC) ? -gamepad1.left_stick_y : gamepad1.left_stick_x;
        double strafeY = (Robot.getInstance().getDrive().getState() == Drive.DriveState.FIELD_CENTRIC) ?
                (Robot.getInstance().side == Constants.Side.BLUE ? -gamepad1.left_stick_x : gamepad1.left_stick_x) :
                -gamepad1.left_stick_y;
        double currentHeading = (Robot.getInstance().getDrive().getState() == Drive.DriveState.FIELD_CENTRIC) ? Robot.getInstance().getOdometry().getPose().heading : 0;
        double rotationPower = gamepad1.right_stick_x;
        Robot.getInstance().getDrive().vectorTo(new Pose(0,0,currentHeading), new Pose(strafeX, strafeY, currentHeading), drivePower, rotationPower);

        //-------------------------------------------- Update --------------------------------------------
        if (odomUpdate.milliseconds() > 10) {
            Robot.getInstance().getOdometry().update();
            odomUpdate.reset();
        }
        Robot.getInstance().outputToTelemetry(telemetry);
        telemetry.update();

    }
}
