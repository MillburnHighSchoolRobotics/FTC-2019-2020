package com.millburnrobotics.skystone.subsystems;

import android.util.Log;

import com.millburnrobotics.skystone.Constants;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.prefs.Preferences;

public class Robot {
    public DcMotorEx lf,lb,rf,rb;
    public DcMotorEx liftL, liftR;
    public DcMotorEx intakeL, intakeR;
    public DcMotorEx er, el, eb;
    public Servo chainBarL, chainBarR, claw;
    public Servo hookL, hookR;
    public Servo sideClawArmLeft, sideClawClawLeft;
    public Servo sideClawArmRight, sideClawClawRight;
    public BNO055IMU bno055IMU;

    private Drive drive = new Drive();
    private Odometry odometry = new Odometry();
    private Intake intake = new Intake();
    private Lift lift = new Lift();
    private ChainBar chainBar = new ChainBar();
    private Hook hook = new Hook();
    private SideClaw sideClawLeft = new SideClaw();
    private SideClaw sideClawRight = new SideClaw();
    private Camera camera = new Camera();
    private IMU imu = new IMU();
    private Subsystem[] subsystems = new Subsystem[] {odometry, drive, intake, lift, chainBar, sideClawLeft, sideClawRight, camera, imu};

    private int cameraMonitorViewerID;

    public HardwareMap hardwareMap;
    public Constants.Side side;
    public Constants.Block block = Constants.Block.NULL;

    public static Robot INSTANCE = null;

    public static Robot getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new Robot();
        }
        return INSTANCE;
    }

    public void init(HardwareMap hardwareMap, boolean auto) {
        this.hardwareMap = hardwareMap;

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

        er = (DcMotorEx) hardwareMap.dcMotor.get(Constants.OdometryConstants._RightEncoder);
        el = (DcMotorEx) hardwareMap.dcMotor.get(Constants.OdometryConstants._LeftEncoder);
        eb = (DcMotorEx) hardwareMap.dcMotor.get(Constants.OdometryConstants._BackEncoder);
        er.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        er.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        el.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        el.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        eb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        eb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intakeL = (DcMotorEx)hardwareMap.dcMotor.get(Constants.IntakeConstants._IntakeLeft);
        intakeR = (DcMotorEx)hardwareMap.dcMotor.get(Constants.IntakeConstants._IntakeRight);
        intakeL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeR.setDirection(DcMotorSimple.Direction.REVERSE);

        liftL = (DcMotorEx)hardwareMap.dcMotor.get(Constants.LiftConstants._LiftLeft);
        liftR = (DcMotorEx)hardwareMap.dcMotor.get(Constants.LiftConstants._LiftRight);

        chainBarL = hardwareMap.servo.get(Constants.ChainBarConstants._ChainBarLeft);
        chainBarR = hardwareMap.servo.get(Constants.ChainBarConstants._ChainBarRight);
        claw = hardwareMap.servo.get(Constants.ChainBarConstants._ChainBarClaw);

        hookL = hardwareMap.servo.get(Constants.HookConstants._FoundationHookLeft);
        hookR = hardwareMap.servo.get(Constants.HookConstants._FoundationHookRight);

        sideClawArmLeft = hardwareMap.servo.get(Constants.SideClawConstants._SideClawArmLeft);
        sideClawClawLeft = hardwareMap.servo.get(Constants.SideClawConstants._SideClawClawLeft);
        sideClawArmRight = hardwareMap.servo.get(Constants.SideClawConstants._SideClawArmRight);
        sideClawClawRight = hardwareMap.servo.get(Constants.SideClawConstants._SideClawClawRight);

        cameraMonitorViewerID = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        if (imu.isDetectingCollisions()) {
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.calibrationDataFile = "BNO055IMUCalibration.json";
            parameters.loggingEnabled = false;
            parameters.loggingTag = "imu 1";
            parameters.accelerationIntegrationAlgorithm = null;
            bno055IMU = hardwareMap.get(BNO055IMU.class, Constants.IMUConstants._IMU);
            bno055IMU.initialize(parameters);
        }

        getDrive().init(auto);
        getOdometry().init(auto);
        getIntake().init(auto);
        getLift().init(auto);
        getChainBar().init(auto);
        getHook().init(auto);
        getSideClawLeft().init(auto);
        getSideClawRight().init(auto);
        getCamera().init(auto);
        getIMU().init(auto);

        getSideClawLeft().setSide(SideClaw.SideClawSide.LEFT);
        getSideClawRight().setSide(SideClaw.SideClawSide.RIGHT);
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
    public SideClaw getSideClawLeft() {
        return sideClawLeft;
    }
    public SideClaw getSideClawRight() {
        return sideClawRight;
    }
    public Camera getCamera() {
        return camera;
    }
    public IMU getIMU() {
        return imu;
    }
    public int getCameraMonitorViewerID() {
        return cameraMonitorViewerID;
    }

    public void update() {
        ElapsedTime updateTimer = new ElapsedTime();

        for (int s = 0; s < subsystems.length; s++) {
            subsystems[s].update();
        }
        Log.d("Update Timer", ""+updateTimer.milliseconds());
        updateTimer.reset();
    }
    public void outputToTelemetry(Telemetry telemetry) {
        for (int s = 0; s < subsystems.length; s++) {
            subsystems[s].outputToTelemetry(telemetry);
        }
    }

    public void savePreference(String key, String value) {
        Preferences prefs = Preferences.userNodeForPackage(Robot.class);
        prefs.put(key, value);
    }
    public String readPreference(String key) {
        Preferences prefs = Preferences.userNodeForPackage(Robot.class);
        return prefs.get(key, "0");
    }
}
