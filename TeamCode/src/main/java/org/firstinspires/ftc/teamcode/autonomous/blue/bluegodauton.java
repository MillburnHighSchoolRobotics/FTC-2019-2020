package org.firstinspires.ftc.teamcode.autonomous.blue;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.heading.ConstantInterpolator;
import com.acmerobotics.roadrunner.path.heading.SplineInterpolator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.mecanum.DriveBase;
import org.firstinspires.ftc.teamcode.drive.mecanum.MohanBot;
import org.firstinspires.ftc.teamcode.threads.PositionMonitor;
import org.firstinspires.ftc.teamcode.threads.ThreadManager;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

@Autonomous(group = "auton")
public class bluegodauton extends LinearOpMode {
    private DcMotorEx intakeL, intakeR, chainBar;
    private Servo clawSquish, clawSpin, foundationHookLeft, foundationHookRight;

    private final double[] squishPos = {0.45,1};
    private final double[] spinPos = {0,0.5};

    private final double[] foundationHookPosLeft = {0.3,0.7};
    private final double[] foundationHookPosRight = {0.7,0.3};

    private final static int[] chainBarPos = {0,500,1200,1500};

    private final double intakePower = 0.9;
    private final static double chainBarPower = 0.6;


    @Override
    public void runOpMode() throws InterruptedException {
        intakeL = (DcMotorEx)hardwareMap.dcMotor.get("intakeL");
        intakeR = (DcMotorEx)hardwareMap.dcMotor.get("intakeR");
        chainBar = (DcMotorEx) hardwareMap.dcMotor.get("chainBar");

        clawSquish = hardwareMap.servo.get("clawSquish");
        clawSpin = hardwareMap.servo.get("clawSpin");
        foundationHookRight = hardwareMap.servo.get("foundationHookRight");
        foundationHookLeft = hardwareMap.servo.get("foundationHookLeft");

        chainBar.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        intakeL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        chainBar.setTargetPosition(chainBarPos[0]);
        chainBar.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        intakeL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        chainBar.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeL.setDirection(REVERSE);

        intakeL.setPower(0);
        intakeR.setPower(0);
        chainBar.setPower(0);

        chainBar.setTargetPositionTolerance(50);

        clawSquish.setPosition(squishPos[0]);
        clawSpin.setPosition(spinPos[1]);
        foundationHookLeft.setPosition(foundationHookPosLeft[0]);
        foundationHookRight.setPosition(foundationHookPosRight[0]);

        ThreadManager manager = ThreadManager.getInstance();
        manager.setHardwareMap(hardwareMap);
        manager.setCurrentAuton(this);
        manager.setupThread("PositionMonitor", PositionMonitor.class);

        DriveBase drive = new MohanBot(hardwareMap);
        drive.setPoseEstimate(new Pose2d(-48,63,0));

        waitForStart();
        if (isStopRequested()) return;


        moveChainbar(1);
        intakeL.setPower(-intakePower);
        intakeR.setPower(-intakePower);

        int skystone = 1;
        switch(skystone) {
            case 1:
                drive.followTrajectorySync(
                        drive.trajectoryBuilder()
                                .splineTo(new Pose2d(-44,32,3*Math.PI/2), new SplineInterpolator(3*Math.PI/2, Math.toRadians(275-45)))
                                .strafeTo(new Vector2d(-48,25))
                                .build()
                );
            case 2:
                drive.followTrajectorySync(
                        drive.trajectoryBuilder()
                                .splineTo(new Pose2d(-52,32,Math.PI/2), new SplineInterpolator(3*Math.PI/2, Math.toRadians(275-45)))
                                .strafeTo(new Vector2d(-56,25))
                                .build()
                );
            case 3:
                drive.followTrajectorySync(
                        drive.trajectoryBuilder()
                                .splineTo(new Pose2d(-60,32,Math.PI/2), new SplineInterpolator(3*Math.PI/2, Math.toRadians(275-45)))
                                .strafeTo(new Vector2d(-64,25))
                                .build()
                );

        }
        moveChainbar(0);
        clawSquish.setPosition(squishPos[1]);
        intakeL.setPower(0);
        intakeR.setPower(0);

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .reverse()
                        .splineTo(new Pose2d(0,42,0), new SplineInterpolator(Math.toRadians(275-45),Math.PI))
                        .splineTo(new Pose2d(51,32,3*Math.PI/2),new SplineInterpolator(Math.PI,Math.PI/2))
                        .back(8)
                        .build()
        );
        foundationHookLeft.setPosition(foundationHookPosLeft[1]);
        foundationHookRight.setPosition(foundationHookPosRight[1]);

        moveChainbar(2);

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .splineTo(new Pose2d(24,48,0))
                        .build()
        );

        clawSquish.setPosition(squishPos[0]);
        moveChainbar(0);
        intakeL.setPower(-intakePower);
        intakeR.setPower(-intakePower);
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .splineTo(new Pose2d(0,42,0),new ConstantInterpolator(Math.PI))
                        .build()
        );
        moveChainbar(1);

        switch(skystone) {
            case 1:
                drive.followTrajectorySync(
                        drive.trajectoryBuilder()
                                .splineTo(new Pose2d(-20,32,0), new SplineInterpolator(Math.PI, Math.toRadians(275-45)))
                                .strafeTo(new Vector2d(-24,25))
                                .build()
                );
            case 2:
                drive.followTrajectorySync(
                        drive.trajectoryBuilder()
                                .splineTo(new Pose2d(-28,32,0),new SplineInterpolator(Math.PI, Math.toRadians(275-45)))
                                .strafeTo(new Vector2d(-32,25))
                                .build()
                );
            case 3:
                drive.followTrajectorySync(
                        drive.trajectoryBuilder()
                                .splineTo(new Pose2d(-36,32,0),new SplineInterpolator(Math.PI, Math.toRadians(275-45)))
                                .strafeTo(new Vector2d(-40,25))
                                .build()
                );

        }
        moveChainbar(0);
        clawSquish.setPosition(squishPos[1]);
        intakeL.setPower(0);
        intakeR.setPower(0);

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .splineTo(new Pose2d(24,48,0))
                        .build()
        );
        moveChainbar(2);
        clawSquish.setPosition(squishPos[0]);

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .forward(48)
                        .build()
        );
        moveChainbar(0);
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .splineTo(new Pose2d(0,42,0),new SplineInterpolator(0,4*Math.PI))
                        .build()
        );
    }

    private void moveChainbar(int whichPos) throws InterruptedException {
        chainBar.setTargetPosition(chainBarPos[whichPos]);
        chainBar.setPower(chainBarPower);
    }
}
