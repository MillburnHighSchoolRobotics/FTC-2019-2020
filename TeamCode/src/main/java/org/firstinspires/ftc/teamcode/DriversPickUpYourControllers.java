package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.MathUtils;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(group = "teleop")
public class DriversPickUpYourControllers extends OpMode {
    final double[] squishPos = {0.45,1};
    final double[] spinPos = {0,0.5};

//    final double chainBarLow = 200;
//    final double chainBarHigh = 2000;

    double intakePower = 0.45;
    double chainBarPower = 0.8;
    double drivePower = 1;

    public DcMotorEx lf;
    public DcMotorEx lb;
    public DcMotorEx rf;
    public DcMotorEx rb;
    public DcMotorEx intakeL;
    public DcMotorEx intakeR;
    public DcMotor chainBar;
    public Servo clawSquish;
    public Servo clawSpin;
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

        intakeL = (DcMotorEx)hardwareMap.dcMotor.get("intakeL");
        intakeR = (DcMotorEx)hardwareMap.dcMotor.get("intakeR");
        chainBar = hardwareMap.dcMotor.get("chainBar");

        clawSquish = hardwareMap.servo.get("clawSquish");
        clawSpin = hardwareMap.servo.get("clawSpin");

        lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        chainBar.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        intakeL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        chainBar.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intakeL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        chainBar.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rf.setDirection(REVERSE);
        rb.setDirection(REVERSE);
        intakeL.setDirection(REVERSE);

        lf.setPower(0);
        lb.setPower(0);
        rf.setPower(0);
        rb.setPower(0);

        intakeL.setPower(0);
        intakeR.setPower(0);
        chainBar.setPower(0);

        clawSquish.setPosition(squishPos[0]);
        clawSpin.setPosition(spinPos[1]);

    }

    @Override
    public void loop() {

        if (gamepad2.x) {
            lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            intakeL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            intakeR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            chainBar.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            lb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            intakeL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            intakeR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            chainBar.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        if (MathUtils.equals(gamepad1.left_trigger, 1, 0.05)) {
            intakeL.setPower(intakePower);
            intakeR.setPower(intakePower);
        } else if (MathUtils.equals(gamepad1.right_trigger, 1, 0.05)) {
            intakeL.setPower(-intakePower);
            intakeR.setPower(-intakePower);
        } else {
            intakeL.setPower(0);
            intakeR.setPower(0);
        }

        if (gamepad1.a) { //close claw
            clawSquish.setPosition(squishPos[1]);
        }

        if (gamepad1.b) { //release claw
            clawSquish.setPosition(squishPos[0]);
        }

        if (gamepad1.x) { //normal claw rotation
            clawSpin.setPosition(spinPos[1]);
        }

        if (gamepad1.y) { //sideways claw rotation
            clawSpin.setPosition(spinPos[0]);
        }

        if (gamepad1.left_bumper) {
            chainBar.setPower(-chainBarPower);
        } else if (gamepad1.right_bumper) {
            chainBar.setPower(chainBarPower);
        } else {
            chainBar.setPower(0);
        }

        if (gamepad1.dpad_up) {
            drivePower = 1;
        } else if (gamepad1.dpad_right) {
            drivePower = 0.8;
        } else if (gamepad1.dpad_down) {
            drivePower = 0.6;
        } else if (gamepad1.dpad_left) {
            drivePower = 0.4;
        }

        double transX = gamepad1.left_stick_x;
        double transY = -gamepad1.left_stick_y;
        double rotX = gamepad1.right_stick_x;
        double translateMag = Math.sqrt(transX*transX + transY*transY);
        double translateTheta = Math.atan2(transY, transX);
        translateTheta = Math.toDegrees(translateTheta);
        if (translateTheta < 0) translateTheta += 360;
        double scale = 0;
        double RF = 0, RB = 0, LF = 0, LB = 0;
        if (!MathUtils.equals(rotX, 0, 0.05)) {
            LF = rotX;
            LB = rotX;
            RF = -rotX;
            RB = -rotX;
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
        lf.setPower(drivePower * LF);
        lb.setPower(drivePower * LB);
        rf.setPower(drivePower * RF);
        rb.setPower(drivePower * RB);
    }
}