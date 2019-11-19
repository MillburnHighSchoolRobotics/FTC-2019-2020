package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.kinematics.Kinematics;
import com.acmerobotics.roadrunner.kinematics.MecanumKinematics;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import java.util.Arrays;
import java.util.List;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;
import static org.firstinspires.ftc.teamcode.robot.GlobalConstants.ROBOT_LENGTH;
import static org.firstinspires.ftc.teamcode.robot.GlobalConstants.ROBOT_WIDTH;
import static org.firstinspires.ftc.teamcode.robot.GlobalConstants.kA;
import static org.firstinspires.ftc.teamcode.robot.GlobalConstants.kStatic;
import static org.firstinspires.ftc.teamcode.robot.GlobalConstants.kV;

public class Drive {
    private DcMotorEx lf, lb, rf, rb;
    private List<DcMotorEx> motors;

    public Drive(DcMotorEx lf, DcMotorEx lb, DcMotorEx rf, DcMotorEx rb) {
        this.lf = lf;
        this.lb = lb;
        this.rf = rf;
        this.rb = rb;

        motors = Arrays.asList(lf,lb,rf,rb);
        for (DcMotorEx motor : motors) {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        rf.setDirection(REVERSE);
        rb.setDirection(REVERSE);
    }
    public void setDriveSignal(DriveSignal driveSignal) {
        List<Double> velocities = MecanumKinematics.robotToWheelVelocities(driveSignal.getVel(), ROBOT_WIDTH, ROBOT_LENGTH);
        List<Double> accelerations = MecanumKinematics.robotToWheelAccelerations(driveSignal.getAccel(), ROBOT_WIDTH, ROBOT_LENGTH);
        List<Double> powers = Kinematics.calculateMotorFeedforward(velocities, accelerations, kV, kA, kStatic);
        setDrivePower(powers.get(0),powers.get(1),powers.get(2),powers.get(3));
    }
    public void setDrivePower(double lfPower, double lbPower, double rbPower, double rfPower) {
        lf.setPower(lfPower);
        lb.setPower(lbPower);
        rf.setPower(rfPower);
        rb.setPower(rbPower);
    }
    public void setDrivePower(double power) {
        setDrivePower(power,power,power,power);
    }
    public void setDrivePower(Pose2d power) {
        List<Double> powers = MecanumKinematics.robotToWheelVelocities(power, ROBOT_WIDTH, ROBOT_LENGTH);
        setDrivePower(powers.get(0),powers.get(1),powers.get(2),powers.get(3));
    }
    public void setDrivePower(double powerLeft, double powerRight) {
        lf.setPower(powerLeft);
        lb.setPower(powerLeft);
        rf.setPower(powerRight);
        rb.setPower(powerRight);
    }
}
