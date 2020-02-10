package com.millburnrobotics.skystone.subsystems;

import com.millburnrobotics.lib.util.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import static com.millburnrobotics.skystone.Constants.LiftConstants.*;

public class Lift extends Subsystem {
    public enum LiftState {
        WORK, FAIL
    }
    private LiftState state;
    private int targetLiftBlock;
    private PIDController liftController = new PIDController(0.01,0,0,0,0);

    @Override
    public void init(boolean auto) {
        Robot.getInstance().liftR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Robot.getInstance().liftR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        targetLiftBlock = 0;
    }

    @Override
    public void outputToTelemetry(Telemetry telemetry) {
        telemetry.addData("Lift position", Robot.getInstance().liftR.getCurrentPosition());
    }

    @Override
    public void update() {

    }

    public void autoLiftToBlock() {
        liftController.setTarget(targetLiftBlock);
    }
    public void autoLiftDown() {
        liftController.setTarget(LIFT_MIN_POS);
    }

    public void manualLiftUp() {
        setLiftPower(LIFT_DAMPEN_POWER * calcLiftDampen(
                Robot.getInstance().liftR.getCurrentPosition()/LIFT_MAX_POS));
        liftController.setTarget(Robot.getInstance().liftR.getCurrentPosition());
    }
    public void manualLiftDown() {
        if (Robot.getInstance().liftR.getCurrentPosition() > LIFT_MIN_POS) {
            setLiftPower(LIFT_RETRACTION_POWER);
        } else {
            setLiftPower(0);
        }
        liftController.setTarget(Robot.getInstance().liftR.getCurrentPosition());
    }

    public void updateLiftPID() {
        double output = liftController.getPIDOutput(Robot.getInstance().liftR.getCurrentPosition());
        setLiftPower(output * LIFT_EXTENSION_POWER);
    }
    public void updateLiftTargetBlock(int block) {
        this.targetLiftBlock = block;
    }

    public void setLiftPosition() {
        liftController.setTarget(LIFT_STONE_POS[targetLiftBlock]);
    }

    public void setLiftPower(double liftPower) {
        Robot.getInstance().liftL.setPower(liftPower);
        Robot.getInstance().liftR.setPower(liftPower);
    }
    public void setState(LiftState state) {
        this.state = state;
    }

    public LiftState getState() {
        return state;
    }

    private double calcLiftDampen(double ratio) {
        double L = 1;
        double b = .95;
        double k = 7;
        double v = 0.9;

        if (ratio < 0) {
            ratio = 0;
        }
        if (ratio >= 0 && ratio < v) {
            return L/(1+Math.pow(Math.E,k*(ratio-b)));
        } else if (ratio >= v && ratio <= 1) {
            return L/(1+Math.pow(Math.E, k*(v-b)));
        }
        return 0;
    }
}
