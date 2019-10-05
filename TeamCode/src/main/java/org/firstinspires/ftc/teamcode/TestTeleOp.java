package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;


import org.firstinspires.ftc.teamcode.autonomous.MathUtils;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

@TeleOp(name = "TEST TELEOP", group = "Testing")
public class TestTeleOp extends OpMode {
    public DcMotorEx lf;
    public DcMotorEx lb;
    public DcMotorEx rf;
    public DcMotorEx rb;
    private double gearCoefficient = 1;
    private double rotateCoefficient = 1;
    public static final int[][] POWER_MATRIX = { //for each of the directions
            {1, 1, 1, 1},
            {1, 0, 0, 1},
            {1, -1, -1, 1},
            {0, -1, -1, 0},
            {-1, -1, -1, -1},
            {-1, 0, 0, -1},
            {-1, 1, 1, -1},
            {0, 1, 1, 0}
    };
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
        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rf.setDirection(REVERSE);
        rb.setDirection(REVERSE);
    }

    @Override
    public void loop() {

        if (gamepad1.x) {
            lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            lb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }




        double transX = gamepad1.left_stick_x;
        double transY = -gamepad1.left_stick_y;
        double rotX = gamepad1.right_stick_x;
        double rotY = gamepad1.right_stick_y;
        double translateMag = Math.sqrt(transX*transX + transY*transY);
        double translateTheta = Math.atan2(transY, transX);
        translateTheta = Math.toDegrees(translateTheta);
        if (translateTheta < 0) translateTheta += 360;
        double scale = 0;
        double RF = 0, RB = 0, LF = 0, LB = 0;
        if (!MathUtils.equals(rotX, 0, 0.05)) {
            LF = rotX * rotateCoefficient;
            LB = rotX * rotateCoefficient;
            RF = -rotX * rotateCoefficient;
            RB = -rotX * rotateCoefficient;
        } else if (!MathUtils.equals(translateMag, 0, 0.05)) {
            double translatePower = translateMag;
            if (translateTheta >= 0 && translateTheta <= 90) { //quadrant 1
                scale = MathUtils.sinDegrees(translateTheta - 45) / MathUtils.cosDegrees(translateTheta - 45);
                LF = translatePower * POWER_MATRIX[0][0];
                LB = translatePower * POWER_MATRIX[0][1] * scale;
                RF = translatePower * POWER_MATRIX[0][2] * scale;
                RB = translatePower * POWER_MATRIX[0][3];
            } else if (translateTheta > 90 && translateTheta <= 180) { //quadrant 2
                translatePower *= -1;
                scale = MathUtils.sinDegrees(translateTheta - 135) / MathUtils.cosDegrees(translateTheta - 135);
                LF = (translatePower * POWER_MATRIX[2][0] * scale);
                LB = (translatePower * POWER_MATRIX[2][1]);
                RF = (translatePower * POWER_MATRIX[2][2]);// * scale);
                RB = (translatePower * POWER_MATRIX[2][3] * scale);
            } else if (translateTheta > 180 && translateTheta <= 270) { //quadrant 3
                scale = MathUtils.sinDegrees(translateTheta - 225) / MathUtils.cosDegrees(translateTheta - 225);
                LF = (translatePower * POWER_MATRIX[4][0]);
                LB = (translatePower * POWER_MATRIX[4][1] * scale);
                RF = (translatePower * POWER_MATRIX[4][2] * scale);
                RB = (translatePower * POWER_MATRIX[4][3]);
            } else if (translateTheta > 270 && translateTheta < 360) { //quadrant 4
                translatePower *= -1;
                scale = MathUtils.sinDegrees(translateTheta - 315) / MathUtils.cosDegrees(translateTheta - 315);
                LF = (translatePower * POWER_MATRIX[6][0] * scale);
                LB = (translatePower * POWER_MATRIX[6][1]);
                RF = (translatePower * POWER_MATRIX[6][2]);
                RB = (translatePower * POWER_MATRIX[6][3] * scale);
            }
        }
        lf.setPower(LF * gearCoefficient);
        lb.setPower(LB * gearCoefficient);
        rf.setPower(RF * gearCoefficient);
        rb.setPower(RB * gearCoefficient);
        telemetry.addData("THETA", translateTheta);
        telemetry.addData("SCALE", scale);
        telemetry.addData("LF", lf.getPower() + " " + lf.getCurrentPosition());
        telemetry.addData("LB", lb.getPower() + " " +  lb.getCurrentPosition());
        telemetry.addData("RF", rf.getPower() + " " + rf.getCurrentPosition());
        telemetry.addData("RB", rb.getPower() + " " + rb.getCurrentPosition());
    }
}