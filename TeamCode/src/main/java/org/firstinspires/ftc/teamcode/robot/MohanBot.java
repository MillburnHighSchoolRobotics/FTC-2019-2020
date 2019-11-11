package org.firstinspires.ftc.teamcode.robot;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.kinematics.Kinematics;
import com.acmerobotics.roadrunner.kinematics.MecanumKinematics;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumConstraints;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.threads.ThreadManager;
import org.firstinspires.ftc.teamcode.util.DashboardUtil;
import org.firstinspires.ftc.teamcode.util.MathUtils;
import org.firstinspires.ftc.teamcode.util.Movement;

import java.util.List;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;
import static org.firstinspires.ftc.teamcode.robot.RobotConstants.BASE_CONSTRAINTS;
import static org.firstinspires.ftc.teamcode.robot.RobotConstants.HEADING_OFFSET;
import static org.firstinspires.ftc.teamcode.robot.RobotConstants.HEADING_PID;
import static org.firstinspires.ftc.teamcode.robot.RobotConstants.ROBOT_LENGTH;
import static org.firstinspires.ftc.teamcode.robot.RobotConstants.ROBOT_WIDTH;
import static org.firstinspires.ftc.teamcode.robot.RobotConstants.TRANSLATION_PID;
import static org.firstinspires.ftc.teamcode.robot.RobotConstants.X_OFFSET;
import static org.firstinspires.ftc.teamcode.robot.RobotConstants.Y_OFFSET;
import static org.firstinspires.ftc.teamcode.robot.RobotConstants.kA;
import static org.firstinspires.ftc.teamcode.robot.RobotConstants.kStatic;
import static org.firstinspires.ftc.teamcode.robot.RobotConstants.kV;

public class MohanBot {
    
    private HardwareMap hardwareMap;
    private DcMotorEx lf, lb, rf, rb, intakeL, intakeR, chainBar;
    private Servo clawSquish, clawSpin, foundationHookLeft, foundationHookRight;

    final double[] squishPos = {0.45,1};
    final double[] foundationHookPosLeft = {0.3,0.7};
    final double[] foundationHookPosRight = {0.7,0.3};
    final static int[] chainBarPos = {0,500,1200};

    final double intakePower = 0.6;
    final double chainBarPower = 0.6;

    public enum Mode {
        IDLE,
        TURN,
        FOLLOW_TRAJECTORY
    }

    private NanoClock clock;
    private Mode mode;

    private FtcDashboard dashboard;

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
        lf = (DcMotorEx)hardwareMap.dcMotor.get("lf");
        lb = (DcMotorEx)hardwareMap.dcMotor.get("lb");
        rf = (DcMotorEx)hardwareMap.dcMotor.get("rf");
        rb = (DcMotorEx)hardwareMap.dcMotor.get("rb");

        intakeL = (DcMotorEx)hardwareMap.dcMotor.get("intakeL");
        intakeR = (DcMotorEx)hardwareMap.dcMotor.get("intakeR");
        chainBar = (DcMotorEx)hardwareMap.dcMotor.get("chainBar");

        clawSquish = hardwareMap.servo.get("clawSquish");
        foundationHookLeft = hardwareMap.servo.get("foundationHookLeft");
        foundationHookRight = hardwareMap.servo.get("foundationHookRight");

        lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        chainBar.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        intakeL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        chainBar.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intakeL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        chainBar.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rf.setDirection(REVERSE);
        rb.setDirection(REVERSE);
        intakeL.setDirection(REVERSE);

        chainBar.setTargetPositionTolerance(50);

        lf.setPower(0);
        lb.setPower(0);
        rf.setPower(0);
        rb.setPower(0);

        intakeL.setPower(0);
        intakeR.setPower(0);
        chainBar.setPower(0);
    }
    private void initRoadRunner() {

        dashboard = FtcDashboard.getInstance();
        dashboard.setTelemetryTransmissionInterval(25);
        
        clock = NanoClock.system();

        mode = Mode.IDLE;

        turnController = new PIDFController(HEADING_PID);
        turnController.setInputBounds(0, 2 * Math.PI);

        constraints = new MecanumConstraints(BASE_CONSTRAINTS, ROBOT_WIDTH, ROBOT_LENGTH);
        follower = new HolonomicPIDVAFollower(TRANSLATION_PID, TRANSLATION_PID, HEADING_PID);
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

    public void setDriveSignal(DriveSignal driveSignal) {
        List<Double> velocities = MecanumKinematics.robotToWheelVelocities(driveSignal.getVel(), ROBOT_WIDTH, ROBOT_LENGTH);
        List<Double> accelerations = MecanumKinematics.robotToWheelAccelerations(driveSignal.getAccel(), ROBOT_WIDTH, ROBOT_LENGTH);
        List<Double> powers = Kinematics.calculateMotorFeedforward(velocities, accelerations, kV, kA, kStatic);
        setDrivePower(powers.get(0),powers.get(1),powers.get(2),powers.get(3));
    }
    public void setDrivePower(double lfPower, double lbPower, double rfPower, double rbPower) {
        lf.setPower(lfPower);
        lb.setPower(rfPower);
        rf.setPower(lbPower);
        rb.setPower(rbPower);
    }
    public void setDrivePower(double power) {
        setDrivePower(power,power,power,power);
    }
    public void setDrivePower(Pose2d power) {
        List<Double> powers = MecanumKinematics.robotToWheelVelocities(power, ROBOT_WIDTH, ROBOT_LENGTH);
        setDrivePower(powers.get(0),powers.get(1),powers.get(2),powers.get(3));
    }

    public TrajectoryBuilder trajectoryBuilder() {
        return new TrajectoryBuilder(getPose(), constraints);
    }

    public void turn(double angle) {
        double heading = getPose().getHeading();
        turnProfile = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(heading, 0, 0, 0),
                new MotionState(heading + angle, 0, 0, 0),
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

    public Pose2d getLastError() {
        switch (mode) {
            case FOLLOW_TRAJECTORY:
                return follower.getLastError();
            case TURN:
                return new Pose2d(0, 0, turnController.getLastError());
            case IDLE:
                return new Pose2d();
        }
        throw new AssertionError();
    }

    public void update() {
        Pose2d currentPose = getPose();
        Pose2d lastError = getLastError();

        TelemetryPacket packet = new TelemetryPacket();
        Canvas fieldOverlay = packet.fieldOverlay();

        packet.put("mode", mode);

        packet.put("x", currentPose.getX());
        packet.put("y", currentPose.getY());
        packet.put("heading", currentPose.getHeading());

        packet.put("xError", lastError.getX());
        packet.put("yError", lastError.getY());
        packet.put("headingError", lastError.getHeading());


        switch (mode) {
            case IDLE:
                setDriveSignal(new DriveSignal());
                break;
            case TURN: {
                double t = clock.seconds() - turnStart;

                MotionState targetState = turnProfile.get(t);
                turnController.setTargetPosition(targetState.getX());

                double targetOmega = targetState.getV();
                double targetAlpha = targetState.getA();
                double correction = turnController.update(currentPose.getHeading(), targetOmega);

                setDriveSignal(new DriveSignal(
                                new Pose2d(0, 0, targetOmega + correction),
                                new Pose2d(0, 0, targetAlpha)
                ));

                if (t >= turnProfile.duration()) {
                    mode = Mode.IDLE;
                    setDriveSignal(new DriveSignal());
                }
                break;
            }
            case FOLLOW_TRAJECTORY: {
                setDriveSignal(follower.update(currentPose));
                Trajectory trajectory = follower.getTrajectory();

                fieldOverlay.setStrokeWidth(1);
                fieldOverlay.setStroke("4CAF50");
                DashboardUtil.drawSampledPath(fieldOverlay, trajectory.getPath());
                fieldOverlay.setStroke("#F44336");

                double t = follower.elapsedTime();

                DashboardUtil.drawRobot(fieldOverlay, trajectory.get(t));
                Log.d("pose", "Target Pose: (" + trajectory.get(t).getX() + "," + trajectory.get(t).getY() + ") - " + Movement.toDegrees(trajectory.get(t).getHeading()));
                Log.d("pose", "Current Pose: (" + currentPose.getX() + "," + currentPose.getY() + ") - " + Movement.toDegrees(currentPose.getHeading()));

                fieldOverlay.setStroke("#3F51B5");
                fieldOverlay.fillCircle(currentPose.getX(), currentPose.getY(), 3);

                if (!follower.isFollowing()) {
                    mode = Mode.IDLE;
                    setDriveSignal(new DriveSignal());
                }
                break;
            }
        }
        dashboard.sendTelemetryPacket(packet);
    }

    private void waitForIdle() {
        while (!Thread.currentThread().isInterrupted() && isBusy()) {
            update();
        }
    }
    private boolean isBusy() {
        return mode != Mode.IDLE;
    }

    public void intakeIn() {
        intakeL.setPower(-intakePower);
        intakeR.setPower(-intakePower);
    }
    public void intakeOut() {
        intakeL.setPower(intakePower);
        intakeR.setPower(intakePower);
    }
    public void intakeStop() {
        intakeL.setPower(0);
        intakeR.setPower(0);
    }
    public void openClaw() {
        clawSquish.setPosition(squishPos[0]);
    }
    public void closeClaw() {
        clawSquish.setPosition(squishPos[1]);
    }
    public void chainBarIn() {
        chainBar.setTargetPosition(chainBarPos[0]);
        chainBar.setPower(chainBarPower);
    }
    public void chainBarUp() {
        chainBar.setTargetPosition(chainBarPos[1]);
        chainBar.setPower(chainBarPower);
    }
    public void chainBarOut() {
        chainBar.setTargetPosition(chainBarPos[2]);
        chainBar.setPower(chainBarPower);
    }
    public void hookDown() {
        foundationHookLeft.setPosition(foundationHookPosLeft[1]);
        foundationHookRight.setPosition(foundationHookPosRight[1]);
    }
    public void hookUp() {
        foundationHookLeft.setPosition(foundationHookPosLeft[0]);
        foundationHookRight.setPosition(foundationHookPosRight[0]);
    }
}
