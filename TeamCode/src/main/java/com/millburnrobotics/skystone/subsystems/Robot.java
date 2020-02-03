package com.millburnrobotics.skystone.subsystems;

import com.millburnrobotics.skystone.Constants;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;

import java.text.SimpleDateFormat;
import java.util.Date;

public class Robot {
    public DcMotorEx lf,lb,rf,rb;
    public DcMotorEx liftL, liftR;
    public DcMotorEx intakeL, intakeR;
    public DcMotorEx er, el, eb;
    public Servo chainBarL, chainBarR, claw;
    public Servo hookL, hookR;
    public Servo sideClawArm, sideClawHook;

    private Drive drive = new Drive();
    private Odometry odometry = new Odometry();
    private Intake intake = new Intake();
    private Lift lift = new Lift();
    private ChainBar chainBar = new ChainBar();
    private SideClaw sideClaw = new SideClaw();

    public HardwareMap hardwareMap;

    public static Robot _INSTANCE = null;

    public static Robot getInstance() { return _INSTANCE; }

    public void init(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;

        Date currentData = new Date();
        SimpleDateFormat format = new SimpleDateFormat("dd.M.yyyy hh:mm:ss");

        RobotLog.a("Robot Init Started at " + format.format(currentData));

        lf = (DcMotorEx)hardwareMap.dcMotor.get(Constants.DriveConstants._LeftFrontMotor);
        lb = (DcMotorEx)hardwareMap.dcMotor.get(Constants.DriveConstants._LeftBackMotor);
        rf = (DcMotorEx)hardwareMap.dcMotor.get(Constants.DriveConstants._RightFrontMotor);
        rb = (DcMotorEx)hardwareMap.dcMotor.get(Constants.DriveConstants._RightBackMotor);

        intakeL = (DcMotorEx)hardwareMap.dcMotor.get(Constants.IntakeConstants._IntakeLeft);
        intakeR = (DcMotorEx)hardwareMap.dcMotor.get(Constants.IntakeConstants._IntakeRight);

        liftL = (DcMotorEx)hardwareMap.dcMotor.get(Constants.LiftConstants._LiftLeft);
        liftR = (DcMotorEx)hardwareMap.dcMotor.get(Constants.LiftConstants._LiftRight);

        er = (DcMotorEx) hardwareMap.dcMotor.get(Constants.DriveConstants._LeftFrontMotor);
        el = (DcMotorEx) hardwareMap.dcMotor.get(Constants.DriveConstants._LeftBackMotor);
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
        sideClawHook = hardwareMap.servo.get(Constants.SideClawConstants._SideClawHook);
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
}
