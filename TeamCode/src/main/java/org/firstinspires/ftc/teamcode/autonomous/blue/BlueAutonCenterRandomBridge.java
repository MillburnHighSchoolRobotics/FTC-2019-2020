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
public class BlueAutonCenterRandomBridge extends LinearOpMode {
    public DcMotorEx intakeL;
    public DcMotorEx intakeR;
    public DcMotor chainBar;
    public Servo clawSquish;
    public Servo clawSpin;

    final double[] squishPos = {0.45,1};
    final double[] spinPos = {0,0.5};

    final double chainBarLow = 200;
    final double chainBarHigh = 2000;
    final double chainBarMid = 700;

    double intakePower = 0.45;
    double chainBarPower = 0.8;


    @Override
    public void runOpMode() throws InterruptedException {
        intakeL = (DcMotorEx)hardwareMap.dcMotor.get("intakeL");
        intakeR = (DcMotorEx)hardwareMap.dcMotor.get("intakeR");
        chainBar = hardwareMap.dcMotor.get("chainBar");

        clawSquish = hardwareMap.servo.get("clawSquish");
        clawSpin = hardwareMap.servo.get("clawSpin");

        chainBar.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        intakeL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        chainBar.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        intakeL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        chainBar.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeR.setDirection(REVERSE);

        intakeL.setPower(0);
        intakeR.setPower(0);
        chainBar.setPower(0);

        clawSquish.setPosition(squishPos[0]);
        clawSpin.setPosition(spinPos[1]);

        ThreadManager manager = ThreadManager.getInstance();
        manager.setHardwareMap(hardwareMap);
        manager.setCurrentAuton(this);
        manager.setupThread("PositionMonitor", PositionMonitor.class);

        DriveBase drive = new MohanBot(hardwareMap);
        drive.setPoseEstimate(new Pose2d(-12, 63, 3*Math.PI/2));

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .splineTo(new Pose2d(-30, 22, 3*Math.PI/2), new SplineInterpolator(3*Math.PI/2, Math.PI))
                        .build()
        );
        intakeL.setPower(intakePower);
        intakeR.setPower(intakePower);

        chainBar.setPower(-chainBarPower);
        chainBar.setTargetPosition((int) chainBarMid);
        Thread.sleep(1000);

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                .forward(4)
                .build()
        );
        Thread.sleep(1000);
        intakeL.setPower(0);
        intakeR.setPower(0);

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .strafeTo(new Vector2d(-34,36))
                        .reverse()
                        .splineTo(new Pose2d(60, 36, Math.PI), new SplineInterpolator(Math.PI,3*Math.PI/2))
                        .reverse()
                        .forward(4.75)
                        .build()
        );

        chainBar.setPower(chainBarPower);
        chainBar.setTargetPosition((int) chainBarHigh);

        Thread.sleep(5000);

        chainBar.setPower(-chainBarPower);
        chainBar.setTargetPosition((int) chainBarLow);

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .reverse()
                        .splineTo(new Pose2d(0, 42, Math.PI))
                        .build()
        );
    }
}
