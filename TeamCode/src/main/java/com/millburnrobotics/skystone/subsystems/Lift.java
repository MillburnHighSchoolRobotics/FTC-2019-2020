package com.millburnrobotics.skystone.subsystems;

import com.millburnrobotics.lib.util.PIDController;
import com.millburnrobotics.skystone.Robot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import static com.millburnrobotics.skystone.Constants.LiftConstants.*;

public class Lift extends Subsystem {
    public enum LiftState {
        WORK, FAIL
    }
    private LiftState state;
    private int targetLiftBlock;
    private PIDController liftController;

    private boolean liftUpReset = false;
    private ElapsedTime liftUpResetTimer = new ElapsedTime();

    private ElapsedTime liftSetBlockTimer = new ElapsedTime();
    private ElapsedTime changeStateTimer = new ElapsedTime();

    @Override
    public void init(boolean auto) {
        liftController = new PIDController(0.01,0,0,0,0);
        targetLiftBlock = 1;
        reset();
    }

    @Override
    public void outputToTelemetry(Telemetry telemetry) {
        telemetry.addData("Lift position", Robot.getInstance().liftR.getCurrentPosition());
        telemetry.addData("Target stone", targetLiftBlock);
    }

    @Override
    public void update() {

    }

    public void reset() {
        Robot.getInstance().liftR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Robot.getInstance().liftR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftController.setTarget(LIFT_MIN_POS);
    }

    public void autoLiftToBlock() {
        liftController.setTarget(LIFT_STONE_POS[targetLiftBlock-1]);
    }
    public void autoLiftDown() {
        if (liftUpResetTimer.milliseconds() > 350) {
            liftUpReset = false;
            setLiftPower(LIFT_DAMPEN_POWER * calcLiftDampenReset(
                    1-(Robot.getInstance().liftR.getCurrentPosition()/LIFT_MAX_POS)));
            liftController.setTarget(LIFT_MIN_POS);
        } else {
            setLiftPower(LIFT_AUTO_RESET_POWER);
        }
    }
    public void autoLiftReset() {
        liftUpReset = true;
        liftUpResetTimer.reset();
    }
    public boolean isLiftUpReset() {
        return liftUpReset;
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
        if (liftSetBlockTimer.milliseconds() > 250) {
            this.targetLiftBlock = block;
            liftSetBlockTimer.reset();
        }
    }
    public int getLiftTargetBlock() {
        return targetLiftBlock;
    }
    public void setLiftPower(double liftPower) {
        Robot.getInstance().liftL.setPower(liftPower);
        Robot.getInstance().liftR.setPower(liftPower);
    }

    private double calcLiftDampen(double ratio) {
        double L = 1;
        double b = .95;
        double k = 9;
        double v = 0.8;

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
    private double calcLiftDampenReset(double ratio) {
        double L = 1;
        double b = .8;
        double k = 3;
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

    public void setState(LiftState state) {
        if (changeStateTimer.milliseconds() > 250) {
            this.state = state;
            changeStateTimer.reset();
        }
    }
    public LiftState getState() {
        return state;
    }
}
