package com.millburnrobotics.skystone;

import com.millburnrobotics.lib.geometry.Pose;
import com.millburnrobotics.lib.util.MathUtils;
import com.millburnrobotics.skystone.subsystems.Hook;
import com.millburnrobotics.skystone.subsystems.Intake;
import com.millburnrobotics.skystone.subsystems.Lift;
import com.millburnrobotics.skystone.subsystems.Robot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import static com.millburnrobotics.skystone.Constants.ChainBarConstants.CHAINBARL_IN_POS;
import static com.millburnrobotics.skystone.Constants.ChainBarConstants.CHAINBARL_OUT_POS;
import static com.millburnrobotics.skystone.Constants.ChainBarConstants.CHAINBAR_INCREMENT;
import static com.millburnrobotics.skystone.Constants.DriveConstants.DRIVE_POWER_HIGH;
import static com.millburnrobotics.skystone.Constants.DriveConstants.DRIVE_POWER_LOW;
import static com.millburnrobotics.skystone.Constants.LiftConstants.LIFT_RAISED_MIN_POS;
import static com.millburnrobotics.skystone.Constants.SideClawConstants.SIDE_ARM_DOWN_POS;
import static com.millburnrobotics.skystone.Constants.SideClawConstants.SIDE_ARM_INCREMENT;
import static com.millburnrobotics.skystone.Constants.SideClawConstants.SIDE_ARM_UP_POS;
import static com.millburnrobotics.skystone.Constants.SideClawConstants.SIDE_CLAW_INCREMENT;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(group = "teleop")
public class TeleOp extends OpMode {
    @Override
    public void init() {
        telemetry.setMsTransmissionInterval(1000);
        Robot.getInstance().init(hardwareMap, false);
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
        if (gamepad2.right_bumper && Robot.getInstance().getSideClaw().getClawPosition() < 1 && Robot.getInstance().getSideClaw().canToggleSideClaw()) {
            Robot.getInstance().getSideClaw().setClawPosition(Robot.getInstance().getSideClaw().getClawPosition() + SIDE_CLAW_INCREMENT);
        } else if (gamepad2.left_bumper && Robot.getInstance().getSideClaw().getClawPosition() > 0 && Robot.getInstance().getSideClaw().canToggleSideClaw()) {
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
        }

        //-------------------------------------------- Claw --------------------------------------------//
        if (gamepad1.a && Robot.getInstance().getChainBar().canToggleClaw()) {
            Robot.getInstance().getChainBar().clawClose();
        } else if (gamepad1.b && Robot.getInstance().getChainBar().canToggleClaw()) {
            Robot.getInstance().getChainBar().clawOpen();
        } else if (gamepad1.y) {
            Robot.getInstance().getChainBar().clawOpen();
        }

        //-------------------------------------------- Movement --------------------------------------------
        double drivePower = (Robot.getInstance().liftR.getCurrentPosition() > LIFT_RAISED_MIN_POS) ? DRIVE_POWER_LOW : DRIVE_POWER_HIGH;
        double strafeX = gamepad1.left_stick_x;
        double strafeY = -gamepad1.left_stick_y;
        double rotationPower = gamepad1.right_stick_x;
        Robot.getInstance().getDrive().vectorTo(new Pose(), new Pose(strafeX, strafeY, 0), drivePower, rotationPower);

        //-------------------------------------------- Lift --------------------------------------------//
        if (gamepad2.dpad_up){
            Robot.getInstance().getLift().updateLiftTargetBlock(4);
        }
        if (gamepad1.dpad_right) {
            if (Robot.getInstance().getLift().getState() == Lift.LiftState.FAIL) {
                Robot.getInstance().getLift().setState(Lift.LiftState.WORK);
            } else {
                Robot.getInstance().getLift().setState(Lift.LiftState.FAIL);
            }
        }
        if (gamepad1.y) {
            Robot.getInstance().getLift().autoLiftDown();
            Robot.getInstance().getChainBar().chainBarUp();
        } else if (gamepad1.dpad_up) {
            Robot.getInstance().getLift().manualLiftUp();
        } else if (gamepad1.dpad_down) {
            Robot.getInstance().getLift().manualLiftDown();
        } else if (Robot.getInstance().getLift().getState() != Lift.LiftState.FAIL){
            if (gamepad1.dpad_left) {
                Robot.getInstance().getLift().autoLiftToBlock();
            }
            Robot.getInstance().getLift().updateLiftPID();
        }

        Robot.getInstance().outputToTelemetry(telemetry);
        telemetry.update();
    }
}
