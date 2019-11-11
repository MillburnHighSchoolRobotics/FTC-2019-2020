package org.firstinspires.ftc.teamcode.robot;

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

import org.firstinspires.ftc.teamcode.subsystems.*;
import org.firstinspires.ftc.teamcode.threads.ThreadManager;
import org.firstinspires.ftc.teamcode.util.MathUtils;

import static org.firstinspires.ftc.teamcode.robot.GlobalConstants.*;

public class MohanBot {
    
    private HardwareMap hardwareMap;
    private Drive drive;
    private ChainBar chainBar;
    private Intake intake;
    private Hook hook;

    public enum Mode {
        IDLE,
        TURN,
        FOLLOW_TRAJECTORY
    }

    private NanoClock clock;
    private Mode mode;

    private PIDFController turnController;
    private MotionProfile turnProfile;
    private DriveConstraints constraints;
    private TrajectoryFollower follower;
    private double turnStart;

    private int threadCount = 0;

    public MohanBot(HardwareMap hardwareMap, LinearOpMode opMode) {
        this.hardwareMap = hardwareMap;
        ThreadManager manager = ThreadManager.getInstance();
        manager.setHardwareMap(hardwareMap);
        manager.setCurrentAuton(opMode);

        initRobotHardware();
        initRoadRunner();
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
    private void initRoadRunner() {
        clock = NanoClock.system();
        mode = Mode.IDLE;

        turnController = new PIDFController(HEADING_PID);
        turnController.setInputBounds(0, 2*Math.PI);

        constraints = new MecanumConstraints(BASE_CONSTRAINTS, ROBOT_WIDTH, ROBOT_LENGTH);
        follower = new HolonomicPIDVAFollower(TRANSLATION_PID, TRANSLATION_PID, HEADING_PID);
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

        double x = ThreadManager.getInstance().getValue("x", Double.class) + X_OFFSET;
        double y = ThreadManager.getInstance().getValue("y", Double.class) + Y_OFFSET;
        double theta = MathUtils.normalize(Math.toRadians(ThreadManager.getInstance().getValue("theta", Double.class)) + HEADING_OFFSET);

        threadCount = threadCount1;
        return new Pose2d(x,y,theta);
    }
    public void setPose(Pose2d offset) {
        X_OFFSET = offset.getX();
        Y_OFFSET = offset.getY();
        HEADING_OFFSET = offset.getHeading();
    }

    public TrajectoryBuilder trajectoryBuilder() {
        return new TrajectoryBuilder(getPose(), constraints);
    }

    public void turn(double angle) {
        double heading = getPose().getHeading();
        turnProfile = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(heading, 0, 0, 0),
                new MotionState(heading+angle, 0, 0, 0),
                constraints.maxAngVel,
                constraints.maxAngAccel,
                constraints.maxAngJerk
        );
        turnStart = clock.seconds();
        mode = Mode.TURN;
        waitForIdle();
    }

    public void followTrajectory(Trajectory trajectory) {
        follower.followTrajectory(trajectory);
        mode = Mode.FOLLOW_TRAJECTORY;
        waitForIdle();
    }

    public void update() {
        switch (mode) {
            case IDLE:
                drive.setDrivePower(0);
                break;
            case TURN: {
                double t = clock.seconds() - turnStart;

                MotionState targetState = turnProfile.get(t);
                turnController.setTargetPosition(targetState.getX());

                double targetOmega = targetState.getV();
                double targetAlpha = targetState.getA();
                double correction = turnController.update(getPose().getHeading(), targetOmega);

                drive.setDriveSignal(new DriveSignal(
                                new Pose2d(0,0,targetOmega + correction),
                                new Pose2d(0,0,targetAlpha)
                ));

                if (t >= turnProfile.duration()) {
                    mode = Mode.IDLE;
                    drive.setDriveSignal(new DriveSignal());
                }
                break;
            }
            case FOLLOW_TRAJECTORY: {
                drive.setDriveSignal(follower.update(getPose()));

                if (!follower.isFollowing()) {
                    mode = Mode.IDLE;
                    drive.setDriveSignal(new DriveSignal());
                }
                break;
            }
        }
    }
    public void waitForIdle() {
        while (!Thread.currentThread().isInterrupted() && isBusy()) {
            update();
        }
    }
    public boolean isBusy() {
        return mode != Mode.IDLE;
    }
}
