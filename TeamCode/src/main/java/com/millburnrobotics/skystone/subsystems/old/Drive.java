package com.millburnrobotics.skystone.subsystems.old;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.Arrays;
import java.util.List;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

public class Drive {
    private DcMotorEx lf,lb,rf,rb;
    private List<DcMotorEx> motors;

    public Drive(DcMotorEx lf, DcMotorEx lb, DcMotorEx rf, DcMotorEx rb) {
        this.lf = lf;
        this.lb = lb;
        this.rf = rf;
        this.rb = rb;

        motors = Arrays.asList(lf,lb,rf,rb);
        for (DcMotorEx motor : motors) {
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        rf.setDirection(REVERSE);
        rb.setDirection(REVERSE);
    }
    public void setDriveVelocity(double[] velocity) {
        lf.setVelocity(velocity[0], AngleUnit.RADIANS);
        lb.setVelocity(velocity[1], AngleUnit.RADIANS);
        rf.setVelocity(velocity[2], AngleUnit.RADIANS);
        rb.setVelocity(velocity[3], AngleUnit.RADIANS);
    }
    public void setDrivePower(double lfPower, double lbPower, double rfPower, double rbPower) {
        lf.setPower(lfPower);
        lb.setPower(lbPower);
        rf.setPower(rfPower);
        rb.setPower(rbPower);
    }
    public void setDrivePower(double[] powers) {
        setDrivePower(powers[0],powers[1],powers[2],powers[3]);
    }
    public void setDrivePower(double power) {
        setDrivePower(power,power,power,power);
    }
    public void setDrivePower(double powerLeft, double powerRight) {
        lf.setPower(powerLeft);
        lb.setPower(powerLeft);
        rf.setPower(powerRight);
        rb.setPower(powerRight);
    }
    public void stop() {
        setDrivePower(0);
    }
}
