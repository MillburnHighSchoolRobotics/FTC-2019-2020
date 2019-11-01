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
public class BlueAutonBlockRandomBaseplate extends LinearOpMode {
    public DcMotorEx intakeL;
    public DcMotorEx intakeR;
    public static DcMotor chainBar;
    public Servo clawSquish;
    public Servo clawSpin;
    public Servo foundationHookLeft, foundationHookRight;

    final double[] squishPos = {0.45,1};
    final double[] spinPos = {0,0.5};
    final double[] foundationHookPos = {0,1};
    final static int[] chainBarPos = {0,1000,2000};

    final double intakePower = 0.45;
    final static double chainBarPower = 0.33;


    @Override
    public void runOpMode() throws InterruptedException {
        intakeL = (DcMotorEx)hardwareMap.dcMotor.get("intakeL");
        intakeR = (DcMotorEx)hardwareMap.dcMotor.get("intakeR");
        chainBar = hardwareMap.dcMotor.get("chainBar");

        clawSquish = hardwareMap.servo.get("clawSquish");
        clawSpin = hardwareMap.servo.get("clawSpin");
        foundationHookRight = hardwareMap.servo.get("foundationHookRight");
        foundationHookLeft = hardwareMap.servo.get("foundationHookLeft");

        chainBar.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        intakeL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        chainBar.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        chainBar.setTargetPosition(chainBarPos[0]);

        intakeL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        chainBar.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeL.setDirection(REVERSE);

        intakeL.setPower(0);
        intakeR.setPower(0);
        chainBar.setPower(0);

        clawSquish.setPosition(squishPos[0]);
        clawSpin.setPosition(spinPos[1]);
        foundationHookRight.setPosition(foundationHookPos[0]);
        foundationHookLeft.setPosition(foundationHookPos[0]);

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
                        .forward(24)
                        .build()
        );
        drive.turnSync(Math.PI);
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .strafeLeft(20)
                        .build()
        );
        moveChainbar(2);
        clawSquish.setPosition(squishPos[1]);
        Thread.sleep(1000);
        moveChainbar(1);
        Thread.sleep(1000);
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .forward(4)
                        .build()
        );
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .strafeRight(20+72)
                        .build()
        );
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .back(4)
                        .build()
        );
        moveChainbar(2);
        clawSquish.setPosition(squishPos[0]);
        Thread.sleep(1000);
        moveChainbar(0);
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .forward(4)
                        .build()
        );
        foundationHookLeft.setPosition(foundationHookPos[1]);
        foundationHookRight.setPosition(foundationHookPos[1]);
        Thread.sleep(1000);
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .forward(48)
                        .build()
        );
        Thread.sleep(1000);
        foundationHookLeft.setPosition(foundationHookPos[0]);
        foundationHookRight.setPosition(foundationHookPos[0]);
        Thread.sleep(1000);
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .strafeRight(64)
                        .build()
        );
    }

    public static void moveChainbar(int whichPos) throws InterruptedException {
        chainBar.setTargetPosition(chainBarPos[whichPos]);
        chainBar.setPower(chainBarPower);
        while(chainBar.isBusy())
            Thread.sleep(10);
        chainBar.setPower(0);
    }
}
