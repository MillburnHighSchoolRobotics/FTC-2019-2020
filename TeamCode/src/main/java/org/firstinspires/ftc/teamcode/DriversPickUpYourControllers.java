package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.MathUtils;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

@TeleOp(group = "teleop")
public class DriversPickUpYourControllers extends OpMode {
    final double[] squishPos = {0.3,1};
    final double[] spinPos = {0.5,0};
    private int currentSpinPos = 1;
    final double[] foundationHookPosLeft = {0.3,0.7};
    final double[] foundationHookPosRight = {0.7,0.3};
    private int currentHook = 0;

    private boolean togglePower = false;

    ElapsedTime toggleSpinTime;
    ElapsedTime toggleDriveSpeed;
    ElapsedTime toggleHook;

    double intakePower = 0.6;
    double chainBarPower = 0.8;
    double drivePower = 1;
    double liftExtensionPower = 1;
    double liftRetractionPower = -0.1;
    double liftHoldPower = 0.25;

    public DcMotorEx lf;
    public DcMotorEx lb;
    public DcMotorEx rf;
    public DcMotorEx rb;
    public DcMotorEx intakeL;
    public DcMotorEx intakeR;
    public DcMotorEx chainBar;
    public DcMotorEx lift;
    public Servo clawSquish;
    public Servo clawSpin;
    public Servo foundationHookLeft;
    public Servo foundationHookRight;

    boolean ascending;

    private final int chainBarIntakePosition = 500;
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
        chainBar = (DcMotorEx)hardwareMap.dcMotor.get("chainBar");
        lift = (DcMotorEx)hardwareMap.dcMotor.get("lift");

        clawSquish = hardwareMap.servo.get("clawSquish");
        clawSpin = hardwareMap.servo.get("clawSpin");
        foundationHookLeft = hardwareMap.servo.get("foundationHookLeft");
        foundationHookRight = hardwareMap.servo.get("foundationHookRight");

        lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        chainBar.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        intakeL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        chainBar.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intakeL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        chainBar.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rf.setDirection(REVERSE);
        rb.setDirection(REVERSE);
        intakeL.setDirection(REVERSE);
        chainBar.setDirection(REVERSE);
//        lift.setDirection(REVERSE);

        chainBar.setTargetPositionTolerance(50);
        lift.setTargetPositionTolerance(25);

        lf.setPower(0);
        lb.setPower(0);
        rf.setPower(0);
        rb.setPower(0);

        intakeL.setPower(0);
        intakeR.setPower(0);
        chainBar.setPower(0);

        clawSquish.setPosition(squishPos[0]);
        clawSpin.setPosition(spinPos[currentSpinPos]);
        foundationHookLeft.setPosition(foundationHookPosLeft[currentHook]);
        foundationHookRight.setPosition(foundationHookPosRight[currentHook]);

        toggleSpinTime = new ElapsedTime();
        toggleDriveSpeed = new ElapsedTime();
        toggleHook = new ElapsedTime();

        ascending = false;
    }

    @Override
    public void loop() {

        if (gamepad2.x) {
            chainBar.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            chainBar.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        if (MathUtils.equals(gamepad1.left_trigger, 1, 0.05)) {
            intakeL.setPower(intakePower);
            intakeR.setPower(intakePower);

        } else if (MathUtils.equals(gamepad1.right_trigger, 1, 0.05)) {
            intakeL.setPower(-intakePower);
            intakeR.setPower(-intakePower);
            clawSquish.setPosition(squishPos[0]);
            //TODO: Redo with potentiometer!
//            if (!MathUtils.equals(chainBar.getCurrentPosition(), CHAINBAR_UP_TICKS, 75)) {
//                chainBar.setTargetPosition(chainBarIntakePosition);
//                chainBar.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                chainBar.setPower(chainBarPower);
//            }
        } else {
            intakeL.setPower(0);
            intakeR.setPower(0);
        }

        //TODO: Redo with potentiometer!
//        if (!chainBar.isBusy() && chainBar.getMode() == DcMotor.RunMode.RUN_TO_POSITION) {
//            chainBar.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            chainBar.setPower(0);
//        }

        if (gamepad1.a) { //close claw
            clawSquish.setPosition(squishPos[1]);
        }

        if (gamepad1.b) { //release claw
            clawSquish.setPosition(squishPos[0]);
        }

        if (gamepad1.y && (toggleSpinTime.milliseconds() > 250)) { //toggle claw rotation
            currentSpinPos = 1 - currentSpinPos;
            clawSpin.setPosition(spinPos[currentSpinPos]);
            toggleSpinTime.reset();
        }
        if (gamepad1.x && (toggleHook.milliseconds() > 250)) { //toggle claw rotation
            currentHook = 1 - currentHook;
            foundationHookLeft.setPosition(foundationHookPosLeft[currentHook]);
            foundationHookRight.setPosition(foundationHookPosRight[currentHook]);
            toggleHook.reset();
        }
        if (gamepad1.left_bumper) {
            chainBar.setPower(chainBarPower);
        } else if (gamepad1.right_bumper) {
            chainBar.setPower(-chainBarPower);
        } else if (chainBar.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
            chainBar.setPower(0);
        }

        if (gamepad1.dpad_up) {
            if (ascending) {
                lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            ascending = true;
            lift.setPower(liftExtensionPower);
        } else if (gamepad1.dpad_down) {
            if (ascending) {
                lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            ascending = false;
            lift.setPower(liftRetractionPower);
        } else if (ascending) {
            lift.setTargetPosition(lift.getCurrentPosition());
            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lift.setPower(liftHoldPower);
        } else {
            lift.setPower(0);
        }

        if (gamepad1.left_stick_button) {
            if (toggleDriveSpeed.milliseconds() > 250)  {
                togglePower = !togglePower;
                if (togglePower) {
                    drivePower = 0.6;
                } else {
                    drivePower = 1;
                }
                toggleDriveSpeed.reset();
            }
        }


        //

        double transX = gamepad1.left_stick_x;
        double transY = -gamepad1.left_stick_y;
        double rotX = gamepad1.right_stick_x;

        double translateMag = Math.sqrt(transX*transX + transY*transY);
        double translateTheta = Math.toDegrees(Math.atan2(transY, transX));
        if (translateTheta < 0) translateTheta += 360;

        double RF = 0, RB = 0, LF = 0, LB = 0;
        if (!MathUtils.equals(translateMag, 0, 0.05)) {
            double translatePower = Math.pow(Math.abs(translateMag),(9/7.0));
            if (translateTheta >= 0 && translateTheta <= 90) { //quadrant 1
                double scale = MathUtils.sinDegrees(translateTheta - 45) / MathUtils.cosDegrees(translateTheta - 45);
                LF = translatePower * POWER_MATRIX[0][0];
                LB = translatePower * POWER_MATRIX[0][1] * scale;
                RF = translatePower * POWER_MATRIX[0][2] * scale;
                RB = translatePower * POWER_MATRIX[0][3];
            } else if (translateTheta > 90 && translateTheta <= 180) { //quadrant 2
                double scale = MathUtils.sinDegrees(translateTheta - 135) / MathUtils.cosDegrees(translateTheta - 135);
                translatePower *= -1;
                LF = (translatePower * POWER_MATRIX[2][0] * scale);
                LB = (translatePower * POWER_MATRIX[2][1]);
                RF = (translatePower * POWER_MATRIX[2][2]);
                RB = (translatePower * POWER_MATRIX[2][3] * scale);
            } else if (translateTheta > 180 && translateTheta <= 270) { //quadrant 3
                double scale = MathUtils.sinDegrees(translateTheta - 225) / MathUtils.cosDegrees(translateTheta - 225);
                LF = (translatePower * POWER_MATRIX[4][0]);
                LB = (translatePower * POWER_MATRIX[4][1] * scale);
                RF = (translatePower * POWER_MATRIX[4][2] * scale);
                RB = (translatePower * POWER_MATRIX[4][3]);
            } else if (translateTheta > 270 && translateTheta < 360) { //quadrant 4
                double scale = MathUtils.sinDegrees(translateTheta - 315) / MathUtils.cosDegrees(translateTheta - 315);
                translatePower *= -1;
                LF = (translatePower * POWER_MATRIX[6][0] * scale);
                LB = (translatePower * POWER_MATRIX[6][1]);
                RF = (translatePower * POWER_MATRIX[6][2]);
                RB = (translatePower * POWER_MATRIX[6][3] * scale);
            }
        }

        LF += rotX;
        LB += rotX;
        RF -= rotX;
        RB -= rotX;

        double[] powerArray = new double[] {LF,LB,RF,RB};
        double maxPower = MathUtils.maxArray(powerArray);
        if (maxPower > 1) {
            for (int x = 0; x < powerArray.length; x++) {
                powerArray[x] = MathUtils.sgn(powerArray[x]) * MathUtils.map(Math.abs(powerArray[x]),0,maxPower,0,1);
            }
        }

        lf.setPower(drivePower * powerArray[0]);
        lb.setPower(drivePower * powerArray[1]);
        rf.setPower(drivePower * powerArray[2]);
        rb.setPower(drivePower * powerArray[3]);
    }
}