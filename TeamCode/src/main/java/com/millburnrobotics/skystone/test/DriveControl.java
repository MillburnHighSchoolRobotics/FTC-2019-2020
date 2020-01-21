package com.millburnrobotics.skystone.test;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.millburnrobotics.skystone.robot.GlobalConstants;
import com.millburnrobotics.skystone.threads.PositionMonitor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.millburnrobotics.skystone.threads.ThreadManager;
import com.millburnrobotics.lib.math.MathUtils;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

@TeleOp(group = "teleop")
public class DriveControl extends OpMode {
    private double drivePower;
    private boolean drivePowerLow = false;
    private ElapsedTime toggleDriveSpeed;

    private DcMotorEx lf;
    private DcMotorEx lb;
    private DcMotorEx rf;
    private DcMotorEx rb;

    DcMotorEx er;
    DcMotorEx el;
    DcMotorEx eb;

    @Override
    public void init() {
        lf = (DcMotorEx)hardwareMap.dcMotor.get("lf");
        lb = (DcMotorEx)hardwareMap.dcMotor.get("lb");
        rf = (DcMotorEx)hardwareMap.dcMotor.get("rf");
        rb = (DcMotorEx)hardwareMap.dcMotor.get("rb");

        lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rf.setDirection(REVERSE);
        rb.setDirection(REVERSE);

        lf.setPower(0);
        lb.setPower(0);
        rf.setPower(0);
        rb.setPower(0);

        toggleDriveSpeed = new ElapsedTime();

        er = (DcMotorEx) hardwareMap.dcMotor.get("chainBar");
        el = (DcMotorEx) hardwareMap.dcMotor.get("intakeR");
        eb = (DcMotorEx) hardwareMap.dcMotor.get("intakeL");
        er.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        er.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        el.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        el.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        eb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        eb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        el.setDirection(DcMotorSimple.Direction.FORWARD);
        er.setDirection(DcMotorSimple.Direction.REVERSE);
        eb.setDirection(DcMotorSimple.Direction.FORWARD);

        ThreadManager manager = ThreadManager.getInstance();
        manager.setHardwareMap(hardwareMap);
        manager.setupThread("PositionMonitor", PositionMonitor.class, new Pose2d(-63,-39,3*Math.PI/2));
    }

    @Override
    public void loop() {
        if (gamepad1.left_stick_button) { // toggle drive speed
            if (toggleDriveSpeed.milliseconds() > 250)  {
                drivePowerLow = !drivePowerLow;
                toggleDriveSpeed.reset();
            }
        }
        if (drivePowerLow) {
            drivePower = GlobalConstants.DRIVE_POWER_LOW;
        } else {
            drivePower = GlobalConstants.DRIVE_POWER_HIGH;
        }

        double strafeX = gamepad1.left_stick_x;
        double strafeY = -gamepad1.left_stick_y;
        double rotation = gamepad1.right_stick_x;
        double[] powerArray = drivePowerArray(strafeX, strafeY, rotation);

        lf.setPower(drivePower*powerArray[0]);
        lb.setPower(drivePower*powerArray[1]);
        rf.setPower(drivePower*powerArray[2]);
        rb.setPower(drivePower*powerArray[3]);

        telemetry.addData("heading", Math.toDegrees(ThreadManager.getInstance().getValue("yaw", Double.class)));
        telemetry.addData("x", ThreadManager.getInstance().getValue("x", Double.class));
        telemetry.addData("y", ThreadManager.getInstance().getValue("y", Double.class));
        telemetry.addData("orientation", Math.toDegrees(ThreadManager.getInstance().getValue("orientation", Double.class)));
        telemetry.addData("rotation", ThreadManager.getInstance().getValue("rotation", Double.class));
        telemetry.addData("count", ThreadManager.getInstance().getValue("count", Double.class));

        telemetry.addData("er", er.getCurrentPosition());
        telemetry.addData("el", el.getCurrentPosition());
        telemetry.addData("eb", eb.getCurrentPosition());

        telemetry.addLine();

        telemetry.addData("Drive Power",drivePower);
        telemetry.addData("lf",lf);
        telemetry.addData("lb",lb);
        telemetry.addData("rf",rf);
        telemetry.addData("rb",rb);
        telemetry.update();
    }

    private double[] drivePowerArray(double strafeX, double strafeY, double rotationPower) {
        double scale, lf = 0, lb = 0, rf = 0, rb = 0;
        double strafeMagnitude = Math.hypot(strafeX,strafeY);
        double absAngle = Math.toDegrees(MathUtils.normalize(Math.atan2(strafeY, strafeX)));
        double strafePower = Math.pow(Math.abs(strafeMagnitude),(9.0/7.0));

        if (!MathUtils.equals(strafeMagnitude, 0, 0.05)) {
            if (absAngle >= 0 && absAngle < 90) {
                scale = MathUtils.tanDegrees(absAngle - 45);
                lf = 1;
                lb = scale;
                rf = scale;
                rb = 1;
            } else if (absAngle >= 90 && absAngle < 180) {
                scale = MathUtils.tanDegrees(absAngle - 135);
                lf = -scale;
                lb = 1;
                rf = 1;
                rb = -scale;
            } else if (absAngle >= 180 && absAngle < 270) {
                scale = MathUtils.tanDegrees(absAngle - 225);
                lf = -1;
                lb = -scale;
                rf = -scale;
                rb = -1;
            } else if (absAngle >= 270 && absAngle < 360) {
                scale = MathUtils.tanDegrees(absAngle - 315);
                lf = scale;
                lb = -1;
                rf = -1;
                rb = scale;
            }
        }

        lf *= strafePower;
        lb *= strafePower;
        rf *= strafePower;
        rb *= strafePower;
        lf += rotationPower;
        lb += rotationPower;
        rf -= rotationPower;
        rb -= rotationPower;

        double[] powerArray = new double[] {lf,lb,rf,rb};
        double maxPower = MathUtils.maxArray(powerArray);
        if (maxPower > 1) {
            for (int x = 0; x < powerArray.length; x++) {
                powerArray[x] = MathUtils.sgn(powerArray[x]) * MathUtils.map(Math.abs(powerArray[x]),0,maxPower,0,1);
            }
        }
        return powerArray;
    }
}