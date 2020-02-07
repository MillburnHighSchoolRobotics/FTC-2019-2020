package com.millburnrobotics.skystone.subsystems;

import com.millburnrobotics.lib.util.PIDController;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import static com.millburnrobotics.skystone.Constants.LiftConstants.*;

public class Lift extends Subsystem {
    public enum LiftState {
        WORK, FAIL
    }
    private LiftState state;
    private int currentBlock;
    private PIDController liftController = new PIDController(0.01,0,0,0,0);

    @Override
    public void init(boolean auto) {
        currentBlock = 0;
    }

    @Override
    public void outputToTelemetry(Telemetry telemetry) {
        telemetry.addData("Lift position", Robot.getInstance().liftL.getCurrentPosition());
    }

    @Override
    public void update() {

    }

    public void autoLiftToBlock() {
        liftController.setTarget(currentBlock);
    }
    public void autoLiftDown() {
        currentBlock = 0;
        liftController.setTarget(currentBlock);
    }

    public void manualLiftUp() {
        setLiftPower(LIFT_DAMPEN_POWER * calcLiftDampen(
                Robot.getInstance().liftL.getCurrentPosition()/LIFT_MAX_POS));
        liftController.setTarget(Robot.getInstance().liftL.getCurrentPosition());
    }
    public void manualLiftDown() {
        setLiftPower(LIFT_RETRACTION_POWER);
        liftController.setTarget(Robot.getInstance().liftL.getCurrentPosition());
    }

    public void updateLiftPID() {
        double output = liftController.getPIDOutput(Robot.getInstance().liftL.getCurrentPosition());
        setLiftPower(output * LIFT_EXTENSION_POWER);
    }

    public void setLiftPosition(int block) {
        currentBlock = block;
    }

    public void setLiftPower(double liftPower) {
        Robot.getInstance().liftL.setPower(liftPower);
        Robot.getInstance().liftR.setPower(-liftPower);
    }
    public void setState(LiftState state) {
        this.state = state;
    }

    public LiftState getState() {
        return state;
    }

    private double calcLiftDampen(double ratio) {
        double L = 1;
        double b = .85;
        double k = 5.5;
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
