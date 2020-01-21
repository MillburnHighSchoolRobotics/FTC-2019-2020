package com.millburnrobotics.skystone.robot.subsystems;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.millburnrobotics.lib.math.MathUtils;
import com.millburnrobotics.skystone.util.PIDController;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;
import static com.millburnrobotics.skystone.Constants.*;

public class ChainBar {
    DcMotorEx chainBar;
    AnalogInput chainBarPot;
    Servo clawClamp;

    PIDController chainBarPID = new PIDController(2.5,0,0);

    public ChainBar(DcMotorEx chainBar, AnalogInput chainBarPot, Servo clawClamp) {
        this.chainBar = chainBar;
        this.chainBarPot = chainBarPot;
        this.clawClamp = clawClamp;

        chainBar.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        chainBar.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        chainBar.setDirection(REVERSE);
    }
    public void openClaw() {
        clawClamp.setPosition(ChainBarConstants.CHAINBAR_CLAW_OPEN_POS);
    }
    public void closeClaw() {
        clawClamp.setPosition(ChainBarConstants.CHAINBAR_CLAW_CLOSE_POS);
    }
    public void chainBarIn() {
        ElapsedTime e = new ElapsedTime();
        chainBarPID.setTarget(ChainBarConstants.CHAINBAR_IN_VOLTAGE);
        while (!MathUtils.equals(chainBarPot.getVoltage(), chainBarPID.getTarget(), 0.1)) {
            chainBar.setPower(chainBarPID.getPIDOutput(chainBarPot.getVoltage()));
            if (e.milliseconds() > 3000) {
                break;
            }
        }
        chainBarStop();
    }
    public void chainBarUp() {
        ElapsedTime e = new ElapsedTime();
        chainBarPID.setTarget(ChainBarConstants.CHAINBAR_UP_VOLTAGE);
        while (!MathUtils.equals(chainBarPot.getVoltage(), chainBarPID.getTarget(), 0.1)) {
            chainBar.setPower(chainBarPID.getPIDOutput(chainBarPot.getVoltage()));
            if (e.milliseconds() > 3000) {
                break;
            }
        }
        chainBarStop();
    }
    public void chainBarOut() {
        ElapsedTime e = new ElapsedTime();
        chainBarPID.setTarget(ChainBarConstants.CHAINBAR_OUT_VOLTAGE);
        while (!MathUtils.equals(chainBarPot.getVoltage(), chainBarPID.getTarget(), 0.1)) {
            chainBar.setPower(chainBarPID.getPIDOutput(chainBarPot.getVoltage()));
            if (e.milliseconds() > 3000) {
                break;
            }
        }
        chainBarStop();
    }
    public void chainBarStop() {
        chainBar.setPower(0);
    }
    public double getVoltage() {
        return chainBarPot.getVoltage();
    }
}
