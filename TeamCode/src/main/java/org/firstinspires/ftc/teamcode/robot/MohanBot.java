package org.firstinspires.ftc.teamcode.robot;

import android.app.Activity;
import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
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

import static org.firstinspires.ftc.teamcode.robot.GlobalConstants.FPS_UPDATE_PERIOD;
import static org.firstinspires.ftc.teamcode.robot.GlobalConstants.TRACK_WIDTH;

public class MohanBot {

    private HardwareMap hardwareMap;
    public Drive drive;
    public ChainBar chainBar;
    public Intake intake;
    public Hook hook;

    private double defaultRotationPower = 0.8;

    private double poseThreshold = 1;
    private double rotationThreshold = Math.toRadians(5);

    private PIDCoefficients rotationPID = new PIDCoefficients(0.000,0.000,0.000);
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

        drive = new Drive(lf,lb,rf,rb);
        chainBar = new ChainBar(chainbar,clawClamp,clawRotate);
        intake = new Intake(intakeLeft,intakeRight);
        hook = new Hook(hookLeft,hookRight);
    }

    public Pose2d getPose() {
        double x = ThreadManager.getInstance().getValue("x", Double.class);
        double y = ThreadManager.getInstance().getValue("y", Double.class);
        double theta = Math.toRadians(ThreadManager.getInstance().getValue("theta", Double.class));

        Pose2d pose = new Pose2d(x,y,theta);
        Log.d("Pose", pose.toString());
        return pose;
    }

    public void moveTo(Pose2d targetPose, double movePower, double targetAngle, double turnPower) {
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
        lf*=movePower;
        lb*=movePower;
        rf*=movePower;
        rb*=movePower;

        while (getPose().vec().distTo(targetPose.vec()) > poseThreshold) {
            drive.setDrivePower(lf, lb, rf, rb);
        }
        drive.stop();
    }

    public void rotate(double angle) throws InterruptedException {
        rotate(angle,defaultRotationPower);
    }
    public void rotate(double angle, double power) throws InterruptedException {
        double targetHeading = MathUtils.normalize(getPose().getHeading()+angle);
        rotateTo(targetHeading,power);
    }
    public void rotateTo(double targetHeading) throws InterruptedException {
        rotateTo(targetHeading,defaultRotationPower);
    }

    public void rotateTo(double targetHeading, double power) throws InterruptedException {
        PIDController pidController = new PIDController(rotationPID.p,rotationPID.i,rotationPID.d,1,targetHeading);

        ElapsedTime time = new ElapsedTime();
        double lastTime = -1;
        while (!shouldStop()) {
            if (Thread.currentThread().isInterrupted()) {
                throw new InterruptedException();
            }
            double currentHeading = getPose().getHeading();
            if (currentHeading - targetHeading > 180) {
                currentHeading -= 2*Math.PI;
            } else if (targetHeading - currentHeading > 180) {
                currentHeading += 2*Math.PI;
            }
            double output = pidController.getPIDOutput(currentHeading);
            if (!MathUtils.equals(currentHeading, targetHeading, rotationThreshold)) {
                if (lastTime == -1) lastTime = time.milliseconds();
                else if (time.milliseconds() - lastTime > 200) {
                    drive.stop();
                    break;
                }
            } else {
                lastTime = -1;
            }
            drive.setDrivePower(-power*output,power*output);

            Thread.sleep(FPS_UPDATE_PERIOD);
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
            drive.setDrivePower(targetVelocity);
        }
        drive.stop();
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
