package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;
import static org.firstinspires.ftc.teamcode.robot.GlobalConstants.*;

public class Intake {
    DcMotorEx intakeL, intakeR;
    public Intake(DcMotorEx intakeL, DcMotorEx intakeR) {
        this.intakeL = intakeL;
        this.intakeR = intakeR;

        intakeL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeL.setDirection(REVERSE);
    }
    public void intakeIn() {
        intakeL.setPower(INTAKE_IN_POWER);
        intakeR.setPower(INTAKE_IN_POWER);
    }
    public void intakeOut() {
        intakeL.setPower(INTAKE_OUT_POWER);
        intakeR.setPower(INTAKE_OUT_POWER);
    }
    public void intakeStop() {
        intakeL.setPower(0);
        intakeR.setPower(0);
    }
}
