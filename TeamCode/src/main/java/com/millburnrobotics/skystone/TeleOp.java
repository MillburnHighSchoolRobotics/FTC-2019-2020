package com.millburnrobotics.skystone;

import com.millburnrobotics.lib.geometry.Pose;
import com.millburnrobotics.lib.util.MathUtils;
import com.millburnrobotics.skystone.subsystems.Hook;
import com.millburnrobotics.skystone.subsystems.Intake;
import com.millburnrobotics.skystone.subsystems.Lift;
import com.millburnrobotics.skystone.subsystems.Robot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import static com.millburnrobotics.skystone.Constants.DriveConstants.DRIVE_POWER_HIGH;
import static com.millburnrobotics.skystone.Constants.DriveConstants.DRIVE_POWER_LOW;

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

        //-------------------------------------------- ChainBar --------------------------------------------//
        if (Robot.getInstance().getIntake().getState() == Intake.IntakeState.INTAKE_IN) {
            Robot.getInstance().getChainBar().chainBarUp();
        } else if (gamepad1.right_bumper) {
            Robot.getInstance().getChainBar().chainBarOut();
        } else if (gamepad1.left_bumper) {
            Robot.getInstance().getChainBar().chainBarIn();
        }

        //-------------------------------------------- Movement --------------------------------------------
        double drivePower = (Robot.getInstance().getLift().getState() == Lift.LiftState.BLOCK) ? DRIVE_POWER_LOW : DRIVE_POWER_HIGH;
        double strafeX = gamepad1.left_stick_x;
        double strafeY = -gamepad1.left_stick_y;
        double rotationPower = gamepad1.right_stick_x;
        Robot.getInstance().getDrive().vectorTo(new Pose(), new Pose(strafeX, strafeY, 0), drivePower, rotationPower);
    }
}
