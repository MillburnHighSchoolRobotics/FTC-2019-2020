package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;
import static org.firstinspires.ftc.teamcode.robot.GlobalConstants.*;

public class ChainBar {
    DcMotorEx chainBar;
    Servo clawClamp, clawRotate;
    public ChainBar(DcMotorEx chainBar, Servo clawClamp, Servo clawRotate) {
        this.chainBar = chainBar;
        this.clawClamp = clawClamp;
        this.clawRotate = clawRotate;

        chainBar.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        chainBar.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        chainBar.setDirection(REVERSE);

        chainBar.setTargetPositionTolerance(50);
    }
    public void openClaw() {
        clawClamp.setPosition(CLAW_OPEN_POS);
    }
    public void closeClaw() {
        clawClamp.setPosition(CLAW_CLOSE_POS);
    }
    public void normalClaw() {
        clawClamp.setPosition(CLAW_NORMAL_POS);
    }
    public void rotateClaw() {
        clawClamp.setPosition(CLAW_ROTATE_POS);
    }
    public void chainBarIn() {
        chainBar.setTargetPosition(CHAINBAR_IN_TICKS);
        chainBar.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        chainBar.setPower(CHAINBAR_POWER);
    }
    public void chainBarUp() {
        chainBar.setTargetPosition(CHAINBAR_UP_TICKS);
        chainBar.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        chainBar.setPower(CHAINBAR_POWER);
    }
    public void chainBarOut() {
        chainBar.setTargetPosition(CHAINBAR_OUT_TICKS);
        chainBar.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        chainBar.setPower(CHAINBAR_POWER);
    }
    public void chainBarStop() {
        chainBar.setPower(0);
    }
    public void chainBarTo(int targetTicks) throws InterruptedException {
        chainBar.setTargetPosition(targetTicks);
        chainBar.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        chainBar.setPower(CHAINBAR_POWER);
        while(chainBar.isBusy()) {
            Thread.sleep(10);
        }
        chainBar.setPower(0);
    }
}
