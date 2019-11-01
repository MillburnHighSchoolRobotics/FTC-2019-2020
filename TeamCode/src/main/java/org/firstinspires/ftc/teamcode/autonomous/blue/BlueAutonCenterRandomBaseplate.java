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

@Deprecated
@Autonomous(group = "auton")
public class BlueAutonCenterRandomBaseplate extends LinearOpMode {
    public DcMotorEx intakeL;
    public DcMotorEx intakeR;
    public DcMotor chainBar;
    public Servo clawSquish;
    public Servo clawSpin;
    public Servo foundationHook;

    final double[] squishPos = {0.45,1};
    final double[] spinPos = {0,0.5};
    final double[] foundationHookPos = {0,1};

    final double chainBarLow = 20;
    final double chainBarHigh = 1890;
    final double chainBarHighMid = 1320;
    final double chainBarMid = 320;

    double intakePower = 0.45;
    double chainBarPower = 0.8;


    @Override
    public void runOpMode() throws InterruptedException {
        intakeL = (DcMotorEx)hardwareMap.dcMotor.get("intakeL");
        intakeR = (DcMotorEx)hardwareMap.dcMotor.get("intakeR");
        chainBar = hardwareMap.dcMotor.get("chainBar");

        clawSquish = hardwareMap.servo.get("clawSquish");
        clawSpin = hardwareMap.servo.get("clawSpin");
        foundationHook = hardwareMap.servo.get("foundationHook");

        chainBar.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        intakeL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        chainBar.setTargetPosition((int)chainBarLow);
        chainBar.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        intakeL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        chainBar.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeL.setDirection(REVERSE);

        intakeL.setPower(0);
        intakeR.setPower(0);
        chainBar.setPower(0);

        clawSquish.setPosition(squishPos[0]);
        clawSpin.setPosition(spinPos[1]);
        foundationHook.setPosition(foundationHookPos[0]);

        ThreadManager manager = ThreadManager.getInstance();
        manager.setHardwareMap(hardwareMap);
        manager.setCurrentAuton(this);
        manager.setupThread("PositionMonitor", PositionMonitor.class);

        DriveBase drive = new MohanBot(hardwareMap);
        drive.setPoseEstimate(new Pose2d(-12, 63, 0));

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .forward(23)
                        .build()
        );
        drive.turnSync(3*Math.PI/2);
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .forward(12)
                        .build()
        );
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .strafeLeft(16)
                        .build()
        );
        intakeL.setPower(intakePower);
        intakeR.setPower(intakePower);

        chainBar.setPower(-chainBarPower);
        chainBar.setTargetPosition((int) chainBarMid);

        clawSquish.setPosition(squishPos[0]);

        Thread.sleep(500);

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                .forward(4)
                .build()
        );
        Thread.sleep(500);
        intakeL.setPower(0);
        intakeR.setPower(0);

        chainBar.setPower(chainBarPower);
        chainBar.setTargetPosition((int) chainBarLow);
        Thread.sleep(500);

        clawSquish.setPosition(squishPos[1]);

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .strafeRight(14)
                        .build()
        );
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .back(82)
                        .build()
        );
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .strafeRight(12)
                        .build()
        );
        drive.turnSync(Math.PI/2);
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .strafeRight(24)
                        .build()
        );
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .strafeLeft(4)
                        .build()
        );
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .back(19)
                        .build()
        );
        foundationHook.setPosition(foundationHookPos[1]);
        Thread.sleep(1000);
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .forward(48)
                        .build()
        );
        Thread.sleep(500);
        foundationHook.setPosition(foundationHookPos[0]);

        chainBar.setPower(-chainBarPower);
        chainBar.setTargetPosition((int) chainBarHigh);
        Thread.sleep(1000);

        clawSquish.setPosition(squishPos[0]);
        Thread.sleep(500);

        chainBar.setPower(chainBarPower);
        chainBar.setTargetPosition((int) chainBarLow);

        Thread.sleep(500);


        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .strafeLeft(64)
                        .build()
        );
    }
}