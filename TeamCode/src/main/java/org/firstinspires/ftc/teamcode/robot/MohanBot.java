package org.firstinspires.ftc.teamcode.robot;

import android.app.Activity;
import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.Path;
import com.acmerobotics.roadrunner.path.PathBuilder;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
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

import static org.firstinspires.ftc.teamcode.robot.GlobalConstants.TURN_POWER;

public class MohanBot {

    private HardwareMap hardwareMap;
    public Drive drive;
    public ChainBar chainBar;
    public Intake intake;
    public Hook hook;

    private double poseThreshold = 2;
    private double purePursuitThreshold = 1;
    private double rotationThreshold = 2;
    private double rotationThresholdMoveDone = 5;
    private double count;

    private PIDCoefficients rotationPID = new PIDCoefficients(0.014,0.002,0.004);

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

        DcMotorEx intakeLeft = (DcMotorEx)hardwareMap.dcMotor.get("intakeL");
        DcMotorEx intakeRight = (DcMotorEx)hardwareMap.dcMotor.get("intakeR");
        DcMotorEx chainbar = (DcMotorEx)hardwareMap.dcMotor.get("chainBar");

        Servo clawClamp = hardwareMap.servo.get("clawSquish");
        Servo clawRotate = hardwareMap.servo.get("clawSpin");
        Servo hookLeft = hardwareMap.servo.get("foundationHookLeft");
        Servo hookRight = hardwareMap.servo.get("foundationHookRight");

        AnalogInput chainBarPot = hardwareMap.get(AnalogInput.class, "chainBarPot");

        drive = new Drive(lf,lb,rf,rb);
        chainBar = new ChainBar(chainbar,chainBarPot, clawClamp,clawRotate);
        intake = new Intake(intakeLeft,intakeRight);
        hook = new Hook(hookLeft,hookRight);
    }

    public Pose2d getPose() {
        double newCount;
        do {
            newCount = ThreadManager.getInstance().getValue("count", Double.class);
        } while (count == newCount);
        count = newCount;

        double x = ThreadManager.getInstance().getValue("x", Double.class);
        double y = ThreadManager.getInstance().getValue("y", Double.class);
        double heading = ThreadManager.getInstance().getValue("yaw", Double.class);

        Pose2d pose = new Pose2d(x,y,heading);
        Log.d("Pose", pose.toString());
        return pose;
    }
    private double[] powerVector(Pose2d currentPose, Vector2d targetVector, double strafePower) {
        double scale, lf = 0, lb = 0, rf = 0, rb = 0;
        double absoluteAngle = Math.atan2(targetVector.getY() - currentPose.getY(), targetVector.getX() - currentPose.getX());
        Log.d("absangle", "" + absoluteAngle);
        double relAngle = Math.toDegrees(MathUtils.normalize(absoluteAngle - currentPose.getHeading()));
        Log.d("relangle", "" + relAngle);

        if (relAngle >= 0 && relAngle < 90) {
            scale = MathUtils.tanDegrees(relAngle - 45);
            lf = 1;
            lb = scale;
            rf = scale;
            rb = 1;
        } else if (relAngle >= 90 && relAngle < 180) {
            scale = MathUtils.tanDegrees(relAngle - 135);
            lf = -scale;
            lb = 1;
            rf = 1;
            rb = -scale;
        } else if (relAngle >= 180 && relAngle < 270) {
            scale = MathUtils.tanDegrees(relAngle - 225);
            lf = -1;
            lb = -scale;
            rf = -scale;
            rb = -1;
        } else if (relAngle >= 270 && relAngle < 360) {
            scale = MathUtils.tanDegrees(relAngle - 315);
            lf = scale;
            lb = -1;
            rf = -1;
            rb = scale;
        }
        lf *= strafePower;
        lb *= strafePower;
        rf *= strafePower;
        rb *= strafePower;
        return new double[] {lf,lb,rf,rb};
    }

    public void strafeTo(Vector2d targetVector, double strafePower) {
        Pose2d currentPose;
        do {
            currentPose = getPose();
            double[] motorPowers = powerVector(currentPose,targetVector,strafePower);
            drive.setDrivePower(motorPowers);
            try {
                Thread.sleep(5);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        } while (currentPose.vec().distTo(targetVector) > poseThreshold);
        drive.setDrivePower(0);
    }

    public void moveTo(Vector2d targetPos, double targetHeading, double strafePower, double rotationPower) {
        targetHeading = Math.toDegrees(MathUtils.normalize(Math.toRadians(targetHeading)));
        PIDController pidController = new PIDController(rotationPID.p,rotationPID.i,rotationPID.d,1,targetHeading);

        ElapsedTime time = new ElapsedTime();
        double lastTime = -1;

        double threshold = rotationThreshold;

        boolean strafe = true;
        while (!shouldStop()) {
            Pose2d currentPose = getPose();
            if (currentPose.vec().distTo(targetPos) <= poseThreshold) {
                strafe = false;
            }
            double lf = 0, lb = 0, rf = 0, rb = 0;
            if (strafe) {
                double[] motorPowers = powerVector(currentPose,targetPos,strafePower);
                lf = motorPowers[0]; lb = motorPowers[1]; rf = motorPowers[2]; rb = motorPowers[3];
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
            if (!strafe) {
                threshold = rotationThresholdMoveDone;
            }
            if (MathUtils.equals(currentHeading, targetHeading, threshold) && !strafe) {
                if (lastTime == -1) lastTime = time.milliseconds();
                else if (time.milliseconds() - lastTime > 50) {
                    drive.stop();
                    break;
                }
            } else {
                lastTime = -1;
            }

            lf -= rotationPower*output;
            lb -= rotationPower*output;
            rf += rotationPower*output;
            rb += rotationPower*output;

            drive.setDrivePower(lf, lb, rf, rb);
        }
    }

    public void rotate(double angle) throws InterruptedException {
        rotate(angle,TURN_POWER);
    }
    public void rotate(double angle, double power) throws InterruptedException {
        double targetHeading = Math.toDegrees(getPose().getHeading())+angle;
        rotateTo(targetHeading,power);
    }
    public void rotateTo(double targetHeading) throws  InterruptedException {
        rotateTo(targetHeading,TURN_POWER);
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

    public PathBuilder path(double angleofattack) {
        return new PathBuilder(new Pose2d(getPose().vec().rotated(-Math.PI/2),angleofattack));
    }
    public PathBuilder path() {
        Pose2d currentPose = getPose();
        return new PathBuilder(new Pose2d(currentPose.vec().rotated(-Math.PI/2),currentPose.getHeading()));
    }

    public void follow(double powerLow, double powerHigh, Path path, double... headingInterpolants) {
        PurePursuitFollower follower = new PurePursuitFollower(path, headingInterpolants);

        boolean strafe = true;
        while (!shouldStop()) {
            Pose2d currentPose = getPose();

            Log.d("pure pursuit", "loop");
            if (MathUtils.equals(currentPose.vec().distTo(follower.end()),0,purePursuitThreshold) && strafe) {
                strafe = false;
                Log.d("pure pursuit", "strafe false");
            }
            double drivePower;
            double[] wheelPowers;
            if (strafe) {
                Vector2d targetPose = follower.updatePosition(currentPose.vec());
                drivePower = follower.strafePower(powerLow,powerHigh, currentPose.vec());
                wheelPowers = powerVector(currentPose, targetPose, drivePower);
            } else {
                drivePower = powerLow;
                wheelPowers = new double[] {0,0,0,0};
            }

            double headingCV = follower.headingCV(currentPose);
            if (headingCV == 0 && !strafe) {
                break;
            }
            wheelPowers[0] -= (1-drivePower)*headingCV;
            wheelPowers[1] -= (1-drivePower)*headingCV;
            wheelPowers[2] += (1-drivePower)*headingCV;
            wheelPowers[3] += (1-drivePower)*headingCV;

            drive.setDrivePower(wheelPowers);
        }
        drive.stop();
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
