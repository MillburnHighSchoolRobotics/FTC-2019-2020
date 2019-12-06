package org.firstinspires.ftc.teamcode.robot;

import android.app.Activity;
import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumConstraints;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.internal.opmode.OpModeManagerImpl;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.robot.subsystems.ChainBar;
import org.firstinspires.ftc.teamcode.robot.subsystems.Drive;
import org.firstinspires.ftc.teamcode.robot.subsystems.Hook;
import org.firstinspires.ftc.teamcode.robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.threads.PositionMonitor;
import org.firstinspires.ftc.teamcode.threads.ThreadManager;
import org.firstinspires.ftc.teamcode.util.MathUtils;
import org.firstinspires.ftc.teamcode.util.PIDController;

import static org.firstinspires.ftc.teamcode.robot.GlobalConstants.TRACK_WIDTH;

public class MohanBot {

    private HardwareMap hardwareMap;
    public Drive drive;
    public ChainBar chainBar;
    public Intake intake;
    public Hook hook;

    private double defaultRotationPower = 0.8;

    private double poseThreshold = 2;
    private double rotationThreshold = 2;
    private double count = 0;

    private PIDCoefficients rotationPID = new PIDCoefficients(0.014,0.002,0.004);
    private DriveConstraints driveConstraints = new DriveConstraints(
            50.0, 40.0, 0.0,
            Math.toRadians(180), Math.toRadians(180), 0.0
    );

    TrajectoryFollower follower = new TrajectoryFollower();

    public MohanBot(HardwareMap hardwareMap, LinearOpMode opMode) {
        this(hardwareMap, opMode, new Pose2d());
    }
    public MohanBot(HardwareMap hardwareMap, LinearOpMode opMode, Pose2d start) {
        this.hardwareMap = hardwareMap;
        ThreadManager manager = ThreadManager.getInstance();
        manager.setHardwareMap(hardwareMap);
        manager.setCurrentAuton(opMode);
        manager.setupThread("PositionMonitor", PositionMonitor.class, start);

        count = ThreadManager.getInstance().getValue("count", Double.class);
        init();
    }
    private void init() {
        DcMotorEx lf = (DcMotorEx)hardwareMap.dcMotor.get("lf");
        DcMotorEx lb = (DcMotorEx)hardwareMap.dcMotor.get("lb");
        DcMotorEx rf = (DcMotorEx)hardwareMap.dcMotor.get("rf");
        DcMotorEx rb = (DcMotorEx)hardwareMap.dcMotor.get("rb");

//        DcMotorEx intakeLeft = (DcMotorEx)hardwareMap.dcMotor.get("intakeL");
//        DcMotorEx intakeRight = (DcMotorEx)hardwareMap.dcMotor.get("intakeR");
//        DcMotorEx chainbar = (DcMotorEx)hardwareMap.dcMotor.get("chainBar");
//
//        Servo clawClamp = hardwareMap.servo.get("clawSquish");
//        Servo clawRotate = hardwareMap.servo.get("clawSpin");
//        Servo hookLeft = hardwareMap.servo.get("foundationHookLeft");
//        Servo hookRight = hardwareMap.servo.get("foundationHookRight");

        drive = new Drive(lf,lb,rf,rb);
//        chainBar = new ChainBar(chainbar,clawClamp,clawRotate);
//        intake = new Intake(intakeLeft,intakeRight);
//        hook = new Hook(hookLeft,hookRight);
    }

    public Pose2d getPose() {
        double newCount;
        do {
            newCount = ThreadManager.getInstance().getValue("count", Double.class);
        } while (count == newCount);
        count = newCount;

        double x = ThreadManager.getInstance().getValue("x", Double.class);
        double y = ThreadManager.getInstance().getValue("y", Double.class);
        double theta = ThreadManager.getInstance().getValue("theta", Double.class);

        Pose2d pose = new Pose2d(x,y,theta);
        Log.d("Pose", pose.toString());
        return pose;
    }

    public void strafeTo(Vector2d targetPos, double strafePower) {
        Pose2d currentPose;
        do {
            currentPose = getPose();
            double scale, lf = 0, lb = 0, rf = 0, rb = 0;
            double absoluteAngle = Math.atan2(targetPos.getY() - currentPose.getY(), targetPos.getX() - currentPose.getX());
            Log.d("absangle", "" + absoluteAngle);
            double relAngle = Math.toDegrees(MathUtils.normalize(absoluteAngle - currentPose.getHeading()));
            Log.d("relangle", "" + relAngle);

            if (relAngle >= 0 && relAngle < 90) {
                scale = Math.tan(Math.toRadians(relAngle - 45));
                lf = 1;
                lb = scale;
                rf = scale;
                rb = 1;
            } else if (relAngle >= 90 && relAngle < 180) {
                scale = Math.tan(Math.toRadians(relAngle - 135));
                lf = -scale;
                lb = 1;
                rf = 1;
                rb = -scale;
            } else if (relAngle >= 180 && relAngle < 270) {
                scale = Math.tan(Math.toRadians(relAngle - 45));
                lf = -1;
                lb = -scale;
                rf = -scale;
                rb = -1;
            } else if (relAngle >= 270 && relAngle < 360) {
                scale = Math.tan(Math.toRadians(relAngle - 45));
                lf = scale;
                lb = -1;
                rf = -1;
                rb = scale;
            }
            lf *= strafePower;
            lb *= strafePower;
            rf *= strafePower;
            rb *= strafePower;
            drive.setDrivePower(lf, lb, rf, rb);
        } while (currentPose.vec().distTo(targetPos) > poseThreshold);
        drive.setDrivePower(0);
    }

    public void moveTo(Vector2d targetPos, double targetHeading, double strafePower, double rotationPower) {
        targetHeading = Math.toDegrees(MathUtils.normalize(Math.toRadians(targetHeading)));
        PIDController pidController = new PIDController(rotationPID.p,rotationPID.i,rotationPID.d,1,targetHeading);


        ElapsedTime time = new ElapsedTime();
        double lastTime = -1;

        boolean strafe = true;
        while (!shouldStop()) {
            Pose2d currentPose = getPose();
            if (currentPose.vec().distTo(targetPos) <= poseThreshold) {
                strafe = false;
            }
            double scale, lf = 0, lb = 0, rf = 0, rb = 0;
            if (strafe) {
                double absoluteAngle = Math.toDegrees(Math.atan2(targetPos.getY() - currentPose.getY(), targetPos.getX() - currentPose.getX()));
                Log.d("absangle", "" + absoluteAngle);
                double relAngle = absoluteAngle - Math.toDegrees(currentPose.getHeading());
                Log.d("relangle", "" + relAngle);
                if (relAngle < 0) {
                    relAngle += 360;
                }

                if (relAngle >= 0 && relAngle < 90) {
                    scale = Math.tan(Math.toRadians(relAngle - 45));
                    lf = 1;
                    lb = scale;
                    rf = scale;
                    rb = 1;
                } else if (relAngle >= 90 && relAngle < 180) {
                    scale = Math.tan(Math.toRadians(relAngle - 135));
                    lf = -scale;
                    lb = 1;
                    rf = 1;
                    rb = -scale;
                } else if (relAngle >= 180 && relAngle < 270) {
                    scale = Math.tan(Math.toRadians(relAngle - 45));
                    lf = -1;
                    lb = -scale;
                    rf = -scale;
                    rb = -1;
                } else if (relAngle >= 270 && relAngle < 360) {
                    scale = Math.tan(Math.toRadians(relAngle - 45));
                    lf = scale;
                    lb = -1;
                    rf = -1;
                    rb = scale;
                }
                lf *= strafePower;
                lb *= strafePower;
                rf *= strafePower;
                rb *= strafePower;
            }

            double currentHeading = Math.toDegrees(currentPose.getHeading());
            if (currentHeading - targetHeading > 180) {
                currentHeading -= 360;
            } else if (targetHeading - currentHeading > 180) {
                currentHeading += 360;
            }
            double output = pidController.getPIDOutput(currentHeading);
            Log.d("turn pid", "output - " + output);
            Log.d("turn pid", "current heading - " + currentHeading);
            Log.d("turn pid", "target heading - " + targetHeading);
            if (MathUtils.equals(currentHeading, targetHeading, rotationThreshold) && !strafe) {
                if (lastTime == -1) lastTime = time.milliseconds();
                else if (time.milliseconds() - lastTime > 50) {
                    drive.stop();
                    break;

                }
            } else {
                lastTime = -1;
            }

            double maxDrivePower = MathUtils.maxArray(new double[] {lf,lb,rf,rb});
            lf -= rotationPower*output;
            lb -= rotationPower*output;
            rf += rotationPower*output;
            rb += rotationPower*output;

            drive.setDrivePower(lf, lb, rf, rb);
        }
    }

    public void rotate(double angle) throws InterruptedException {
        rotate(angle,defaultRotationPower);
    }
    public void rotate(double angle, double power) throws InterruptedException {
        double targetHeading = Math.toDegrees(getPose().getHeading())+angle;
        rotateTo(targetHeading,power);
    }
    public void rotateTo(double targetHeading) throws InterruptedException {
        rotateTo(targetHeading,defaultRotationPower);
    }

    public void rotateTo(double targetHeading, double power) throws InterruptedException {
        targetHeading = Math.toDegrees(MathUtils.normalize(Math.toRadians(targetHeading)));
        PIDController pidController = new PIDController(rotationPID.p,rotationPID.i,rotationPID.d,1,targetHeading);

        ElapsedTime time = new ElapsedTime();
        double lastTime = -1;
        while (!shouldStop()) {
            if (Thread.currentThread().isInterrupted()) {
                throw new InterruptedException();
            }

            double currentHeading = Math.toDegrees(getPose().getHeading());
            if (currentHeading - targetHeading > 180) {
                currentHeading -= 360;
            } else if (targetHeading - currentHeading > 180) {
                currentHeading += 360;
            }
            double output = pidController.getPIDOutput(currentHeading);
            Log.d("turn pid", "output - " + output);
            Log.d("turn pid", "current heading - " + currentHeading);
            Log.d("turn pid", "target heading - " + targetHeading);
            if (MathUtils.equals(currentHeading, targetHeading, rotationThreshold)) {
                if (lastTime == -1) lastTime = time.milliseconds();
                else if (time.milliseconds() - lastTime > 50) {
                    drive.stop();
                    Log.d("turn pid", "STOP");
                    break;
                }
            } else {
                lastTime = -1;
            }
            drive.setDrivePower(-power*output,power*output);

            Thread.sleep(5);
        }
    }

    public TrajectoryBuilder trajectoryBuilder() {
        return trajectoryBuilder(getPose().getHeading());
    }
    public TrajectoryBuilder trajectoryBuilder(double heading) {
        return new TrajectoryBuilder(new Pose2d(getPose().vec(),heading), new MecanumConstraints(driveConstraints,TRACK_WIDTH));
    }

    public void followTrajectory(Trajectory trajectory) {
        follower.followTrajectory(trajectory);
        while (follower.activeTrajectory()) {
            Pose2d targetVelocity = follower.update(getPose());
            drive.setDriveVelocity(targetVelocity);
        }
        Pose2d end = follower.getEndPose();
        moveTo(end.vec(),Math.toDegrees(end.getHeading()),0.8,0.2);
    }

    public PIDCoefficients getTurnPID() {
        return rotationPID;
    }
    public void setTurnPID(PIDCoefficients pid) {
        rotationPID = pid;
    }
    public static boolean shouldStop() {
        Activity currActivity = AppUtil.getInstance().getActivity();
        OpModeManagerImpl manager = OpModeManagerImpl.getOpModeManagerOfActivity(currActivity);
        OpMode currentOpMode = manager.getActiveOpMode();
        return currentOpMode instanceof LinearOpMode &&
                ((LinearOpMode) currentOpMode).isStarted() &&
                ((LinearOpMode) currentOpMode).isStopRequested();
    }
}
