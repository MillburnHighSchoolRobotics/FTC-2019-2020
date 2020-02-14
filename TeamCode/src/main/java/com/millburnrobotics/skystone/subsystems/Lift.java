package com.millburnrobotics.skystone.subsystems;

import com.millburnrobotics.lib.util.PIDController;
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

    private boolean clawOpenReset = false;
    private boolean liftUpReset = false;
    private ElapsedTime clawOpenResetTimer = new ElapsedTime(); // waits for claw to open
    private ElapsedTime liftUpResetTimer = new ElapsedTime(); // after claw open, lift up before down

    private ElapsedTime liftSetBlockTimer = new ElapsedTime(); // for 2nd driver
    private ElapsedTime changeStateTimer = new ElapsedTime();

    @Override
    public void init(boolean auto) {
        liftController = new PIDController(0.01,0,0,0,0);
        targetLiftBlock = 0;
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
        liftController.setTarget(0);
    }

    public void autoLiftToBlock() {
        liftController.setTarget(LIFT_STONE_POS[targetLiftBlock]);
    }
    public void autoLiftDown() {
        if (clawOpenResetTimer.milliseconds() > 200 && !clawOpenReset) {
            clawOpenReset = true;
            liftUpResetTimer.reset();
        }
        if (clawOpenReset) {
            if (liftUpResetTimer.milliseconds() > 200) {
                liftUpReset = false;
                clawOpenReset = false;
                liftController.setTarget(LIFT_MIN_POS);
            } else {
                setLiftPower(LIFT_AUTO_RESET_POWER);
            }
        }
    }
    public void autoLiftReset() {
        clawOpenReset = false;
        liftUpReset = true;
        clawOpenResetTimer.reset();
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
