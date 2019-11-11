package org.firstinspires.ftc.teamcode.test;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robot.MohanBot;
import org.firstinspires.ftc.teamcode.threads.PositionMonitor;
import org.firstinspires.ftc.teamcode.threads.ThreadManager;
import org.firstinspires.ftc.teamcode.util.BarkerClass;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;


@Autonomous(group = "auton")
public class BarkerClassTest extends LinearOpMode {
    public DcMotorEx intakeL;
    public DcMotorEx intakeR;
    public DcMotor chainBar;
    public Servo clawSquish;
    public Servo clawSpin;
    public Servo foundationHookLeft, foundationHookRight;

    final double[] squishPos = {0.45,1};
    final double[] spinPos = {0,0.5};
    final double[] foundationHookPos = {1,0};
    final static int[] chainBarPos = {0,750,1200,1500};

    final double intakePower = 0.9;
    final static double chainBarPower = 0.6;

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
        chainBar.setTargetPosition(chainBarPos[0]);
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
        foundationHookLeft.setPosition(foundationHookPos[1]);
        foundationHookRight.setPosition(foundationHookPos[0]);

        ThreadManager manager = ThreadManager.getInstance();
        manager.setHardwareMap(hardwareMap);
        manager.setCurrentAuton(this);
        manager.setupThread("PositionMonitor", PositionMonitor.class);

        MohanBot drive = new MohanBot(hardwareMap,this);

        waitForStart();

        if (isStopRequested()) return;

        BarkerClass barker = new BarkerClass(hardwareMap);

        drive.followTrajectory(
                drive.trajectoryBuilder()
                        .forward(6)
                        .build()
        );

        barker.setupCam();

        Thread.sleep(5000);

        Log.d("BarkerClass Block Pos:",barker.getPos() + "");
    }
}
