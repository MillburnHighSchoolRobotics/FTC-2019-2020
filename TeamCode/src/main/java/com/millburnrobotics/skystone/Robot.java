package com.millburnrobotics.skystone;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.ValueProvider;
import com.acmerobotics.dashboard.config.variable.BasicVariable;
import com.acmerobotics.dashboard.config.variable.CustomVariable;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.millburnrobotics.lib.geometry.Pose;
import com.millburnrobotics.skystone.subsystems.Camera;
import com.millburnrobotics.skystone.subsystems.ChainBar;
import com.millburnrobotics.skystone.subsystems.Drive;
import com.millburnrobotics.skystone.subsystems.Hook;
import com.millburnrobotics.skystone.subsystems.IMU;
import com.millburnrobotics.skystone.subsystems.Intake;
import com.millburnrobotics.skystone.subsystems.Lift;
import com.millburnrobotics.skystone.subsystems.Odometry;
import com.millburnrobotics.skystone.subsystems.SideClaw;
import com.millburnrobotics.skystone.subsystems.Subsystem;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.prefs.Preferences;

import static com.millburnrobotics.skystone.Constants.DriveConstants.BOT_WIDTH;

public class Robot {
    public DcMotorEx lf,lb,rf,rb;
    public DcMotorEx liftL, liftR;
    public DcMotorEx intakeL, intakeR;
    public DcMotorEx er, el, eb;
    public Servo chainBarL, chainBarR, claw;
    public Servo hookL, hookR;
    public Servo sideClawArmL, sideClawArmR;
    public Servo sideClawClawL, sideClawClawR;
    public Servo capstone;
    public BNO055IMU bno055IMU;

    public ElapsedTime timer;

    private Drive drive = new Drive();
    private Odometry odometry = new Odometry();
    private Intake intake = new Intake();
    private Lift lift = new Lift();
    private ChainBar chainBar = new ChainBar();
    private Hook hook = new Hook();
    private SideClaw sideClaw = new SideClaw();
    private Camera camera = new Camera();
    private IMU imu = new IMU();
    private Subsystem[] subsystems = new Subsystem[] {odometry, drive, intake, lift, chainBar, sideClaw, camera, imu};

    private int cameraMonitorViewerID;

    public HardwareMap hardwareMap;
    public FtcDashboard dashboard;

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

        sideClawArmL = hardwareMap.servo.get(Constants.SideClawConstants._SideClawArmL);
        sideClawClawL = hardwareMap.servo.get(Constants.SideClawConstants._SideClawClawL);
        sideClawArmR = hardwareMap.servo.get(Constants.SideClawConstants._SideClawArmR);
        sideClawClawR = hardwareMap.servo.get(Constants.SideClawConstants._SideClawClawR);

        capstone = hardwareMap.servo.get(Constants.ChainBarConstants._Capstone);

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
        getSideClaw().init(auto);
        getCamera().init(auto);
        getIMU().init(auto);

        timer = new ElapsedTime();

        dashboard = FtcDashboard.getInstance();
        dashboard.setTelemetryTransmissionInterval(25);

        addConfiguration();
    }
    public void addConfiguration() {
        String catName = getClass().getSimpleName();
        CustomVariable catVar = (CustomVariable) dashboard.getConfigRoot().getVariable(catName);
        if (catVar == null) {
            catVar = new CustomVariable();
            dashboard.getConfigRoot().putVariable(catName, catVar);
        }

        CustomVariable ff = new CustomVariable();
        ff.putVariable("kv", new BasicVariable<>(new ValueProvider<Double>() {
            @Override public Double get() { return drive.kv; }
            @Override public void set(Double value) { drive.kv = value; }
        }));
        ff.putVariable("ka", new BasicVariable<>(new ValueProvider<Double>() {
            @Override public Double get() { return drive.ka; }
            @Override public void set(Double value) { drive.ka = value; }
        }));
        ff.putVariable("kp", new BasicVariable<>(new ValueProvider<Double>() {
            @Override public Double get() { return drive.kp; }
            @Override public void set(Double value) { drive.kp = value; }
        }));
        catVar.putVariable("FF_CONST", ff);
        Robot.getInstance().dashboard.updateConfig();
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

        TelemetryPacket packet = new TelemetryPacket();
        packet.fieldOverlay().setStroke("#3F51B5");
        packet.fieldOverlay().strokeCircle(odometry.getX(), odometry.getY(),BOT_WIDTH/2.0);
        Pose v = odometry.getPose().polar(BOT_WIDTH/2.0);
        double x1 = odometry.getX() + v.getX() / 2, y1 = odometry.getY() + v.getY() / 2;
        double x2 = odometry.getX() + v.getX(), y2 = odometry.getY() + v.getY();
        packet.fieldOverlay().strokeLine(x1, y1, x2, y2);
        dashboard.sendTelemetryPacket(packet);
    }
    public void outputToTelemetry(Telemetry telemetry) {
        TelemetryPacket packet = new TelemetryPacket();
        for (int s = 0; s < subsystems.length; s++) {
            subsystems[s].outputToTelemetry(telemetry, packet);
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
