package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.PIDController;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;
import static org.firstinspires.ftc.teamcode.robot.GlobalConstants.*;

public class ChainBar {
    DcMotorEx chainBar;
    AnalogInput chainBarPot;
    Servo clawClamp, clawRotate;

    PIDController chainBarPID = new PIDController(1.5,0,0);

    public ChainBar(DcMotorEx chainBar, AnalogInput chainBarPot, Servo clawClamp, Servo clawRotate) {
        this.chainBar = chainBar;
        this.chainBarPot = chainBarPot;
        this.clawClamp = clawClamp;
        this.clawRotate = clawRotate;

        chainBar.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        chainBar.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        chainBar.setDirection(REVERSE);
    }
    public void openClaw() {
        clawClamp.setPosition(CLAW_OPEN_POS);
    }
    public void closeClaw() {
        clawClamp.setPosition(CLAW_CLOSE_POS);
    }
    public void normalClaw() {
        clawRotate.setPosition(CLAW_NORMAL_POS);
    }
    public void rotatedClaw() {
        clawRotate.setPosition(CLAW_ROTATE_POS);
    }
    public void chainBarIn() {
        chainBarPID.setTarget(CHAINBAR_IN_VOLTAGE);
        chainBar.setPower(chainBarPID.getPIDOutput(chainBarPot.getVoltage()));
    }
    public void chainBarUp() {
        chainBarPID.setTarget(CHAINBAR_UP_VOLTAGE);
        chainBar.setPower(chainBarPID.getPIDOutput(chainBarPot.getVoltage()));
    }
    public void chainBarOut() {
        chainBarPID.setTarget(CHAINBAR_OUT_VOLTAGE);
        chainBar.setPower(chainBarPID.getPIDOutput(chainBarPot.getVoltage()));
    }
    public void chainBarStop() {
        chainBar.setPower(0);
    }
}
