package com.millburnrobotics.skystone.subsystems;

import android.util.Log;

import com.millburnrobotics.lib.geometry.Pose;
import com.millburnrobotics.skystone.Constants;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.text.SimpleDateFormat;
import java.util.Date;

public class Robot {
    public DcMotorEx lf,lb,rf,rb;
    public DcMotorEx liftL, liftR;
    public DcMotorEx intakeL, intakeR;
    public DcMotorEx er, el, eb;
    public Servo chainBarL, chainBarR, claw;
    public Servo hookL, hookR;
    public Servo sideClawArm, sideClawClaw;

    private Drive drive = new Drive();
    private Odometry odometry = new Odometry();
    private Intake intake = new Intake();
    private Lift lift = new Lift();
    private ChainBar chainBar = new ChainBar();
    private Hook hook = new Hook();
    private SideClaw sideClaw = new SideClaw();
    private Subsystem[] subsystems = new Subsystem[] {drive, odometry, intake, lift, chainBar, sideClaw};

    public HardwareMap hardwareMap;
    public Constants.SIDE side;

    public Pose pose;

    public static Robot INSTANCE = null;

    public static Robot getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new Robot();
        }
        return INSTANCE;
    }

    public void init(HardwareMap hardwareMap, boolean auto) {
        this.hardwareMap = hardwareMap;

        Date currentData = new Date();
        SimpleDateFormat format = new SimpleDateFormat("dd.M.yyyy hh:mm:ss");

        RobotLog.a("Robot Init Started at " + format.format(currentData));

        lf = (DcMotorEx)hardwareMap.dcMotor.get(Constants.DriveConstants._LeftFrontMotor);
        lb = (DcMotorEx)hardwareMap.dcMotor.get(Constants.DriveConstants._LeftBackMotor);
        rf = (DcMotorEx)hardwareMap.dcMotor.get(Constants.DriveConstants._RightFrontMotor);
        rb = (DcMotorEx)hardwareMap.dcMotor.get(Constants.DriveConstants._RightBackMotor);

        lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        intakeL = (DcMotorEx)hardwareMap.dcMotor.get(Constants.IntakeConstants._IntakeLeft);
        intakeR = (DcMotorEx)hardwareMap.dcMotor.get(Constants.IntakeConstants._IntakeRight);

        intakeL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        liftL = (DcMotorEx)hardwareMap.dcMotor.get(Constants.LiftConstants._LiftLeft);
        liftR = (DcMotorEx)hardwareMap.dcMotor.get(Constants.LiftConstants._LiftRight);


        er = (DcMotorEx) hardwareMap.dcMotor.get(Constants.DriveConstants._LeftFrontMotor);
        el = (DcMotorEx) hardwareMap.dcMotor.get(Constants.DriveConstants._RightBackMotor);
        eb = (DcMotorEx) hardwareMap.dcMotor.get(Constants.DriveConstants._RightFrontMotor);
        er.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        er.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        el.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        el.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        eb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        eb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        el.setDirection(DcMotorSimple.Direction.FORWARD);
        er.setDirection(DcMotorSimple.Direction.REVERSE);
        eb.setDirection(DcMotorSimple.Direction.FORWARD);


        chainBarL = hardwareMap.servo.get(Constants.ChainBarConstants._ChainBarLeft);
        chainBarR = hardwareMap.servo.get(Constants.ChainBarConstants._ChainBarRight);
        claw = hardwareMap.servo.get(Constants.ChainBarConstants._ChainBarClaw);


        hookL = hardwareMap.servo.get(Constants.HookConstants._FoundationHookLeft);
        hookR = hardwareMap.servo.get(Constants.HookConstants._FoundationHookRight);


        sideClawArm = hardwareMap.servo.get(Constants.SideClawConstants._SideClawArm);
        sideClawClaw = hardwareMap.servo.get(Constants.SideClawConstants._SideClawClaw);


        getDrive().init(auto);
        getOdometry().init(auto);
        getIntake().init(auto);
        getLift().init(auto);
        getChainBar().init(auto);
        getHook().init(auto);
        getSideClaw().init(auto);
    }
    public Drive getDrive() {
        return drive;
    }
    public Odometry getOdometry() {
        return odometry;
    }
    public Intake getIntake() {
        return intake;
    }
    public Lift getLift() {
        return lift;
    }
    public ChainBar getChainBar() {
        return chainBar;
    }
    public Hook getHook() {
        return hook;
    }
    public SideClaw getSideClaw() {
        return sideClaw;
    }

    public Pose getPose() {
        return this.getOdometry().getPose();
    }

    public void update() {
        ElapsedTime updateTimer = new ElapsedTime();

        for (int s = 0; s < subsystems.length; s++) {
            subsystems[s].update();
        }
        pose = odometry.getPose();

        Log.d("Update Timer", ""+updateTimer.milliseconds());
        updateTimer.reset();
    }
    public void outputToTelemetry(Telemetry telemetry) {
        for (int s = 0; s < subsystems.length; s++) {
            subsystems[s].outputToTelemetry(telemetry);
        }
    }
}
