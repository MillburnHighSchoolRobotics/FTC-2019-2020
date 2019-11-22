package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import java.util.Arrays;
import java.util.List;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;
import static org.firstinspires.ftc.teamcode.robot.GlobalConstants.TRACK_WIDTH;
import static org.firstinspires.ftc.teamcode.robot.GlobalConstants.VELOCITY_TO_POWER;
import static org.firstinspires.ftc.teamcode.robot.GlobalConstants.WHEEL_BASE;

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
    public void setDrivePower(Pose2d velocity) {
        double k = (WHEEL_BASE+TRACK_WIDTH)/2;
        double[] targetWheelVelocity = new double[] {
                velocity.getX()-velocity.getY()-k*velocity.getHeading(),
                velocity.getX()+velocity.getY()-k*velocity.getHeading(),
                velocity.getX()+velocity.getY()+k*velocity.getHeading(),
                velocity.getX()-velocity.getY()+k*velocity.getHeading()
        };
        double[] wheelPowers = new double[targetWheelVelocity.length];
        for (int v = 0; v < wheelPowers.length; v++) {
            wheelPowers[v] = targetWheelVelocity[v] * VELOCITY_TO_POWER;
        }
        setDrivePower(wheelPowers);
    }
    public void setDrivePower(double[] powers) {
        setDrivePower(powers[0],powers[1],powers[2],powers[3]);
    }
    public void setDrivePower(double lfPower, double lbPower, double rfPower, double rbPower) {
        lf.setPower(lfPower);
        lb.setPower(lbPower);
        rf.setPower(rfPower);
        rb.setPower(rbPower);
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
