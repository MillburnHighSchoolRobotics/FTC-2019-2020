package com.millburnrobotics.skystone.subsystems;

import com.millburnrobotics.lib.util.PIDController;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import static com.millburnrobotics.skystone.Constants.LiftConstants.LIFT_EXTENSION_POWER;

public class Lift extends Subsystem {
    public enum LiftState {
        DOWN, BLOCK
    }
    private LiftState state;
    private int currentBlock;
    private PIDController liftController = new PIDController();

    @Override
    public void init(boolean auto) {
        currentBlock = 0;
    }

    @Override
    public void outputToTelemetry(Telemetry telemetry) {
//        telemetry.addData("LiftState", state);
    }

    @Override
    public void update() {

    }

    public void liftToBlock(int block) {
        state = LiftState.BLOCK;
        currentBlock = block;
        liftController.setTarget(currentBlock);
    }
    public void liftDown() {
        state = LiftState.DOWN;
        currentBlock = 0;
        liftController.setTarget(currentBlock);
    }
    public void updateLiftPID() {
        double output = liftController.getPIDOutput(Robot.getInstance().liftL.getCurrentPosition());
        Robot.getInstance().liftL.setPower(output * LIFT_EXTENSION_POWER);
        Robot.getInstance().liftR.setPower(output * LIFT_EXTENSION_POWER);
    }
    public LiftState getState() {
        return state;
    }
}
