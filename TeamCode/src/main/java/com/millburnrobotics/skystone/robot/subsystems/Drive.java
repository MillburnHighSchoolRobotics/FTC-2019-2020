package com.millburnrobotics.skystone.robot.subsystems;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.millburnrobotics.skystone.robot.GlobalConstants;
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
    public void setDriveVelocity(Pose2d velocity) {
        double k = (GlobalConstants.WHEEL_BASE+ GlobalConstants.TRACK_WIDTH)/2;
        double[] linearVelocity = new double[] {
                velocity.getY()+velocity.getX()-k*velocity.getHeading(),
                velocity.getY()-velocity.getX()-k*velocity.getHeading(),
                velocity.getY()-velocity.getX()+k*velocity.getHeading(),
                velocity.getY()+velocity.getX()+k*velocity.getHeading()
        };

        double[] angularVelocity = new double[linearVelocity.length];
        for (int v = 0; v < angularVelocity.length; v++) {
            angularVelocity[v] = linearVelocity[v] / GlobalConstants.WHEEL_RADIUS;
        }
        setDriveVelocity(angularVelocity);

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
