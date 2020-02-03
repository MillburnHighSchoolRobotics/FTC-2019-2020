package com.millburnrobotics.skystone.subsystems.old;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import static com.millburnrobotics.skystone.Constants.IntakeConstants.INTAKE_IN_POWER;
import static com.millburnrobotics.skystone.Constants.IntakeConstants.INTAKE_OUT_POWER;

public class Intake {
    DcMotorEx intakeL, intakeR;
    public Intake(DcMotorEx intakeL, DcMotorEx intakeR) {
        this.intakeL = intakeL;
        this.intakeR = intakeR;

        intakeL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public void intakeIn() {
        intakeL.setPower(-INTAKE_IN_POWER);
        intakeR.setPower(INTAKE_IN_POWER);
    }
    public void intakeOut() {
        intakeL.setPower(-INTAKE_OUT_POWER);
        intakeR.setPower(INTAKE_OUT_POWER);
    }
    public void intakeStop() {
        intakeL.setPower(0);
        intakeR.setPower(0);
    }
}
