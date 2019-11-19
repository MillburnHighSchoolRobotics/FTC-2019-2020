package org.firstinspires.ftc.teamcode.robot;

import android.util.Log;

import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumConstraints;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robot.subsystems.*;
import org.firstinspires.ftc.teamcode.threads.PositionMonitor;
import org.firstinspires.ftc.teamcode.threads.ThreadManager;
import org.firstinspires.ftc.teamcode.util.MathUtils;

import static org.firstinspires.ftc.teamcode.robot.GlobalConstants.*;

public class TobNahom {

    private HardwareMap hardwareMap;
    private Drive drive;
    private ChainBar chainBar;
    private Intake intake;
    private Hook hook;

    private double poseThreshold = 1;
    private double rotateThreshold = 5;

    private DriveConstraints constraints = new MecanumConstraints(BASE_CONSTRAINTS, ROBOT_WIDTH, ROBOT_LENGTH);
    private int threadCount = 0;

    public TobNahom(HardwareMap hardwareMap, LinearOpMode opMode) {
        this.hardwareMap = hardwareMap;
        ThreadManager manager = ThreadManager.getInstance();
        manager.setHardwareMap(hardwareMap);
        manager.setCurrentAuton(opMode);
        manager.setupThread("PositionMonitor", PositionMonitor.class);

        initRobotHardware();
    }
    private void initRobotHardware() {
        DcMotorEx lf = (DcMotorEx)hardwareMap.dcMotor.get("lf");
        DcMotorEx lb = (DcMotorEx)hardwareMap.dcMotor.get("lb");
        DcMotorEx rf = (DcMotorEx)hardwareMap.dcMotor.get("rf");
        DcMotorEx rb = (DcMotorEx)hardwareMap.dcMotor.get("rb");

        DcMotorEx intakeLeft = (DcMotorEx)hardwareMap.dcMotor.get("intakeL");
        DcMotorEx intakeRight = (DcMotorEx)hardwareMap.dcMotor.get("intakeR");
        DcMotorEx chainbar = (DcMotorEx)hardwareMap.dcMotor.get("chainBar");

        Servo clawClamp = hardwareMap.servo.get("clawSquish");
        Servo clawRotate = hardwareMap.servo.get("clawSpin");
        Servo hookLeft = hardwareMap.servo.get("foundationHookLeft");
        Servo hookRight = hardwareMap.servo.get("foundationHookRight");

        drive = new Drive(lf,lb,rf,rb);
        chainBar = new ChainBar(chainbar,clawClamp,clawRotate);
        intake = new Intake(intakeLeft,intakeRight);
        hook = new Hook(hookLeft,hookRight);
    }
    public Drive getDrive() {
        return drive;
    }
    public ChainBar getChainBar() {
        return chainBar;
    }
    public Intake getIntake() {
        return intake;
    }
    public Hook getHook() {
        return hook;
    }
    public Pose2d getPose() {
        int threadCount1;
        do {
            threadCount1 = ThreadManager.getInstance().getValue("count", Integer.class);
        } while (threadCount == threadCount1);

        double x = ThreadManager.getInstance().getValue("x", Double.class);
        double y = ThreadManager.getInstance().getValue("y", Double.class);
        double theta = MathUtils.normalize(Math.toRadians(ThreadManager.getInstance().getValue("theta", Double.class)));

        threadCount = threadCount1;
        Pose2d pose = new Pose2d(x,y,theta);
        Log.d("Pose", pose.toString());
        return pose;
    }
    public void setPose(Pose2d offset) {
        X_OFFSET = offset.getX();
        Y_OFFSET = offset.getY();
        HEADING_OFFSET = offset.getHeading();
        PENDING_OFFSET = true;
    }

    public void moveTo(Pose2d targetPose, double power) {
        double absoluteAngle =  Math.toDegrees(Math.atan2(targetPose.getY()-getPose().getY(), targetPose.getX()-getPose().getX()));
        double relAngle = absoluteAngle-getPose().getHeading();
        if (relAngle < 0) {
            relAngle += 360;
        }
        double scale, lf=0, lb=0, rf=0, rb=0;
        if (relAngle >= 0 && relAngle < 90) {
            scale = Math.round(Math.tan(Math.toRadians(relAngle-45)));
            lf = 1;
            lb = scale;
            rf = scale;
            rb = 1;
        } else if (relAngle >= 90 && relAngle < 180) {
            scale = Math.round(Math.tan(Math.toRadians(relAngle-135)));
            lf = -scale;
            lb = 1;
            rf = 1;
            rb = -scale;
        } else if (relAngle >= 180 && relAngle < 270) {
            scale = Math.round(Math.tan(Math.toRadians(relAngle-45)));
            lf = -1;
            lb = -scale;
            rf = -scale;
            rb = -1;
        } else if (relAngle >= 270 && relAngle < 360) {
            scale = Math.round(Math.tan(Math.toRadians(relAngle-45)));
            lf = scale;
            lb = -1;
            rf = -1;
            rb = scale;
        }
        lf*=power;
        lb*=power;
        rf*=power;
        rb*=power;
        double dist = Math.hypot(getPose().getX()-targetPose.getX(), getPose().getY()-targetPose.getY());
        while (dist > poseThreshold) {
            getDrive().setDrivePower(lf, lb, rf, rb);
        }
        getDrive().setDrivePower(0);
    }

    public void rotate(double power, double degrees) {
        double targetHeading = Math.toDegrees(getPose().getHeading()) + degrees;
        rotateTo(power, targetHeading);
    }

    public void rotateTo(double power, double targetHeading) {
        if (targetHeading < 0) {
            targetHeading += 360;
        } else if (targetHeading >= 360) {
            targetHeading -= 360;
        }
        while (!MathUtils.equals(getPose().getHeading(), targetHeading, rotateThreshold)) {
            if (getPose().getHeading() < targetHeading) {
                getDrive().setDrivePower(power, -power);
            } else {
                getDrive().setDrivePower(-power, power);
            }
        }
        getDrive().setDrivePower(0);
    }
}
