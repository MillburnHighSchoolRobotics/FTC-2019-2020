//package com.millburnrobotics.skystone.subsystems.old;
//
//import android.app.Activity;
//import android.util.Log;
//
//import com.millburnrobotics.lib.geometry.Pose;
//import com.millburnrobotics.lib.math.MathUtils;
//import com.millburnrobotics.skystone.Constants;
//import com.millburnrobotics.skystone.threads.ThreadManager;
//import com.millburnrobotics.skystone.util.PIDController;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.hardware.AnalogInput;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.hardware.PIDCoefficients;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.util.ElapsedTime;
//import com.qualcomm.robotcore.util.RobotLog;
//
//import org.firstinspires.ftc.robotcore.internal.opmode.OpModeManagerImpl;
//import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
//
//import java.text.SimpleDateFormat;
//import java.util.Date;
//
//public class Robot {
//
//    public DcMotorEx lf, lb, rf, rb;
//    public DcMotorEx intakeL, intakeR;
//    public DcMotorEx liftL, liftR;
//    public Drive drive;
//    public ChainBar chainBar;
//    public Intake intake;
//    public Hook hook;
//    public SideClaw sideClaw;
//
//    private double purePursuitThreshold = 2;
//    private double rotationThreshold = 2;
//
//    private double count;
//
//    private PIDCoefficients rotationPID = new PIDCoefficients(0.014,0.002,0.004);
//
//    private void init(HardwareMap hardwareMap, boolean auto) {
//        Date currentData = new Date();
//        SimpleDateFormat format = new SimpleDateFormat("dd.M.yyyy hh:mm:ss");
//
//        RobotLog.a("Robot Init Started at " + format.format(currentData));
//
//        DcMotorEx lf = (DcMotorEx)hardwareMap.dcMotor.get(Constants.DriveConstants.kLeftFrontMotor);
//        DcMotorEx lb = (DcMotorEx)hardwareMap.dcMotor.get(Constants.DriveConstants.kLeftBackMotor);
//        DcMotorEx rf = (DcMotorEx)hardwareMap.dcMotor.get(Constants.DriveConstants.kRightFrontMotor);
//        DcMotorEx rb = (DcMotorEx)hardwareMap.dcMotor.get(Constants.DriveConstants.kRightBackMotor);
//
//        DcMotorEx intakeLeft = (DcMotorEx)hardwareMap.dcMotor.get(Constants.IntakeConstants.kIntakeL);
//        DcMotorEx intakeRight = (DcMotorEx)hardwareMap.dcMotor.get(Constants.IntakeConstants.kIntakeR);
//
//        DcMotorEx chainbar = (DcMotorEx)hardwareMap.dcMotor.get(Constants.ChainBarConstants.kChainBar);
//        Servo chainBarClawClamp = hardwareMap.servo.get(Constants.ChainBarConstants.kChainBarClamp);
//
//        Servo sideClawBar = hardwareMap.servo.get(Constants.SideClawConstants.kSideClawBar);
//        Servo sideClawClamp = hardwareMap.servo.get(Constants.SideClawConstants.kSideClawClamp);
//
//        Servo hookLeft = hardwareMap.servo.get(Constants.HookConstants.kHookL);
//        Servo hookRight = hardwareMap.servo.get(Constants.HookConstants.kHookR);
//
//        AnalogInput chainBarPot = hardwareMap.get(AnalogInput.class, "chainBarPot");
//
//        drive = new Drive(lf,lb,rf,rb);
//        chainBar = new ChainBar(chainbar,chainBarPot, chainBarClawClamp);
//        intake = new Intake(intakeLeft,intakeRight);
//        hook = new Hook(hookLeft,hookRight);
//        sideClaw = new SideClaw(sideClawBar, sideClawClamp);
//    }
//    public Pose getPose() {
//        double newCount;
//        do {
//            newCount = ThreadManager.getInstance().getValue("count", Double.class);
//        } while (count == newCount);
//        count = newCount;
//
//        double x = ThreadManager.getInstance().getValue("x", Double.class);
//        double y = ThreadManager.getInstance().getValue("y", Double.class);
//        double heading = ThreadManager.getInstance().getValue("yaw", Double.class);
//
//        Pose pose = new Pose(x,y,heading);
//        Log.d("Pose", pose.toString());
//        return pose;
//    }
//    private double[] powerVector(Pose currentPose, Pose targetVector, double strafePower) {
//        double scale, lf = 0, lb = 0, rf = 0, rb = 0;
//        double absoluteAngle = Math.atan2(targetVector.getY() - currentPose.getY(), targetVector.getX() - currentPose.getX());
//        Log.d("absangle", "" + absoluteAngle);
//        double relAngle = Math.toDegrees(MathUtils.normalize(absoluteAngle - currentPose.getHeading()));
//        Log.d("relangle", "" + relAngle);
//
//        if (relAngle >= 0 && relAngle < 90) {
//            scale = MathUtils.tanDegrees(relAngle - 45);
//            lf = 1;
//            lb = scale;
//            rf = scale;
//            rb = 1;
//        } else if (relAngle >= 90 && relAngle < 180) {
//            scale = MathUtils.tanDegrees(relAngle - 135);
//            lf = -scale;
//            lb = 1;
//            rf = 1;
//            rb = -scale;
//        } else if (relAngle >= 180 && relAngle < 270) {
//            scale = MathUtils.tanDegrees(relAngle - 225);
//            lf = -1;
//            lb = -scale;
//            rf = -scale;
//            rb = -1;
//        } else if (relAngle >= 270 && relAngle < 360) {
//            scale = MathUtils.tanDegrees(relAngle - 315);
//            lf = scale;
//            lb = -1;
//            rf = -1;
//            rb = scale;
//        }
//        lf *= strafePower;
//        lb *= strafePower;
//        rf *= strafePower;
//        rb *= strafePower;
//        return new double[] {lf,lb,rf,rb};
//    }
//
//    public void rotateTo(double targetHeading, double power) throws InterruptedException {
//        rotateTo(targetHeading, power, rotationThreshold);
//    }
//    public void rotateTo(double targetHeading, double power, double rotThresh) throws InterruptedException {
//        targetHeading = Math.toDegrees(MathUtils.normalize(Math.toRadians(targetHeading)));
//        PIDController pidController = new PIDController(rotationPID.p,rotationPID.i,rotationPID.d,1,targetHeading);
//
//        ElapsedTime time = new ElapsedTime();
//        double lastTime = -1;
//        while (!shouldStop()) {
//            if (Thread.currentThread().isInterrupted()) {
//                throw new InterruptedException();
//            }
//
//            double currentHeading = Math.toDegrees(getPose().getHeading());
//            if (currentHeading - targetHeading > 180) {
//                currentHeading -= 360;
//            } else if (targetHeading - currentHeading > 180) {
//                currentHeading += 360;
//            }
//            double output = pidController.getPIDOutput(currentHeading);
//            Log.d("turn pid", "output - " + output);
//            Log.d("turn pid", "current heading - " + currentHeading);
//            Log.d("turn pid", "target heading - " + targetHeading);
//            if (MathUtils.equals(currentHeading, targetHeading, rotThresh)) {
//                if (lastTime == -1) lastTime = time.milliseconds();
//                else if (time.milliseconds() - lastTime > 50) {
//                    drive.stop();
//                    Log.d("turn pid", "STOP");
//                    break;
//                }
//            } else {
//                lastTime = -1;
//            }
//            drive.setDrivePower(-power*output,power*output);
//            Thread.sleep(5);
//        }
//    }
//
////    public void follow(double powerLow, double powerHigh, Path path) {
////        PurePursuitFollower follower = new PurePursuitFollower(path);
////
////        while (!shouldStop()) {
////            Pose currentPose = getPose();
////
////            if (MathUtils.equals(currentPose.distTo(follower.path.end()),0,purePursuitThreshold)) {
////                break;
////            }
////
////            Pose targetPose = follower.update(currentPose);
////            double drivePower = follower.strafePower(powerLow,powerHigh, currentPose);
////            double[] wheelPowers = powerVector(currentPose, targetPose, drivePower);
////
////            drive.setDrivePower(wheelPowers);
////        }
////        drive.stop();
////    }
//    public static boolean shouldStop() {
//        Activity currActivity = AppUtil.getInstance().getActivity();
//        OpModeManagerImpl manager = OpModeManagerImpl.getOpModeManagerOfActivity(currActivity);
//        OpMode currentOpMode = manager.getActiveOpMode();
//        return currentOpMode instanceof LinearOpMode &&
//                ((LinearOpMode) currentOpMode).isStarted() &&
//                ((LinearOpMode) currentOpMode).isStopRequested();
//    }
//}
