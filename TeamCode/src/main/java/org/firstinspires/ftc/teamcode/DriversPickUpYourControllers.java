package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.MathUtils;
import org.firstinspires.ftc.teamcode.util.PIDController;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;
import static org.firstinspires.ftc.teamcode.robot.GlobalConstants.CHAINBAR_HIGH_POWER;
import static org.firstinspires.ftc.teamcode.robot.GlobalConstants.CHAINBAR_IN_VOLTAGE;
import static org.firstinspires.ftc.teamcode.robot.GlobalConstants.CHAINBAR_MAX_VOLTAGE;
import static org.firstinspires.ftc.teamcode.robot.GlobalConstants.CHAINBAR_UP_VOLTAGE;
import static org.firstinspires.ftc.teamcode.robot.GlobalConstants.CHAINBAR_CLAW_CLOSE_POS;
import static org.firstinspires.ftc.teamcode.robot.GlobalConstants.CHAINBAR_CLAW_OPEN_POS;
import static org.firstinspires.ftc.teamcode.robot.GlobalConstants.DRIVE_POWER_HIGH;
import static org.firstinspires.ftc.teamcode.robot.GlobalConstants.DRIVE_POWER_LOW;
import static org.firstinspires.ftc.teamcode.robot.GlobalConstants.INTAKE_IN_POWER;
import static org.firstinspires.ftc.teamcode.robot.GlobalConstants.INTAKE_OUT_POWER;
import static org.firstinspires.ftc.teamcode.robot.GlobalConstants.LEFT_HOOK_DOWN_POS;
import static org.firstinspires.ftc.teamcode.robot.GlobalConstants.LEFT_HOOK_UP_POS;
import static org.firstinspires.ftc.teamcode.robot.GlobalConstants.LIFT_DAMPEN_POWER;
import static org.firstinspires.ftc.teamcode.robot.GlobalConstants.LIFT_EXTENSION_POWER;
import static org.firstinspires.ftc.teamcode.robot.GlobalConstants.LIFT_HOLD_POWER;
import static org.firstinspires.ftc.teamcode.robot.GlobalConstants.LIFT_MAX_POS;
import static org.firstinspires.ftc.teamcode.robot.GlobalConstants.LIFT_RAISED_MIN_POS;
import static org.firstinspires.ftc.teamcode.robot.GlobalConstants.LIFT_RETRACTION_POWER;
import static org.firstinspires.ftc.teamcode.robot.GlobalConstants.LIFT_STONE_POS;
import static org.firstinspires.ftc.teamcode.robot.GlobalConstants.RIGHT_HOOK_DOWN_POS;
import static org.firstinspires.ftc.teamcode.robot.GlobalConstants.RIGHT_HOOK_UP_POS;

@TeleOp(group = "teleop")
public class DriversPickUpYourControllers extends OpMode {
    private double drivePower = 1;
    private double chainBarPower;
    private double liftPower;

    private int targetLiftHeight = 1;

    private boolean drivePowerLow = false;
    private boolean intakeIn = false;
    private boolean hookUp = true;
    private boolean liftUpdatedPos = true;
    private boolean liftRaised = false;
    private boolean liftDown = false;
    private boolean liftAutoMode = false;

    private ElapsedTime toggleDriveSpeed;
    private ElapsedTime toggleClawSpin;
    private ElapsedTime timerLiftUp;
    private ElapsedTime timerLiftDown;
    private ElapsedTime toggleHook;

    private DcMotorEx lf;
    private DcMotorEx lb;
    private DcMotorEx rf;
    private DcMotorEx rb;
    private DcMotorEx intakeL;
    private DcMotorEx intakeR;
    private DcMotorEx chainBar;
    private DcMotorEx lift;
    private Servo clawSquish;
    private Servo foundationHookLeft;
    private Servo foundationHookRight;
    private AnalogInput chainBarPot;

    private PIDController chainBarPID = new PIDController(2.25,0,0,0,CHAINBAR_UP_VOLTAGE);
    private PIDController liftPID = new PIDController(0.01,0,0,0,0);

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

        clawSquish = hardwareMap.servo.get("chainBarClawClamp");
        foundationHookLeft = hardwareMap.servo.get("foundationHookLeft");
        foundationHookRight = hardwareMap.servo.get("foundationHookRight");

        chainBarPot = hardwareMap.get(AnalogInput.class, "chainBarPot");

        lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        chainBar.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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
        chainBar.setDirection(REVERSE);

        lf.setPower(0);
        lb.setPower(0);
        rf.setPower(0);
        rb.setPower(0);

        intakeL.setPower(0);
        intakeR.setPower(0);
        chainBar.setPower(0);

        clawSquish.setPosition(CHAINBAR_CLAW_OPEN_POS);
        foundationHookRight.setPosition(RIGHT_HOOK_UP_POS);
        foundationHookLeft.setPosition(LEFT_HOOK_UP_POS);

        toggleDriveSpeed = new ElapsedTime();
        toggleClawSpin = new ElapsedTime();
        toggleHook = new ElapsedTime();
        timerLiftUp = new ElapsedTime();
        timerLiftDown = new ElapsedTime();

        chainBarPID.setTarget(chainBarPot.getVoltage());
    }

    @Override
    public void loop() {

        if (gamepad2.right_stick_button) { // reset
            chainBar.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            chainBar.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }


        //-------------------------------------------- Intake --------------------------------------------//

        if (MathUtils.equals(gamepad1.left_trigger, 1, 0.05)) { // intake out
            intakeL.setPower(-INTAKE_OUT_POWER);
            intakeR.setPower(INTAKE_OUT_POWER);
            intakeIn = false;
        } else if (MathUtils.equals(gamepad1.right_trigger, 1, 0.05)) { // intake in
            intakeL.setPower(-INTAKE_IN_POWER);
            intakeR.setPower(INTAKE_IN_POWER);
            clawSquish.setPosition(CHAINBAR_CLAW_OPEN_POS);
            chainBarPID.setTarget(CHAINBAR_UP_VOLTAGE);
            chainBarPower = chainBarPID.getPIDOutput(chainBarPot.getVoltage());
            intakeIn = true;
        } else { // stop intake
            intakeL.setPower(0);
            intakeR.setPower(0);
            intakeIn = false;
        }


        //-------------------------------------------- Lift --------------------------------------------

//        if (gamepad2.dpad_up && (timerLiftUp.milliseconds() > 100)) { // auto raise lift
//            if (targetLiftHeight < LIFT_STONE_POS.length) {
//                targetLiftHeight++;
//                timerLiftUp.reset();
//            }
//        } else if (gamepad2.dpad_down && (timerLiftDown.milliseconds() > 100)) { // auto lower lift
//            if (targetLiftHeight > 1) {
//                targetLiftHeight--;
//                timerLiftDown.reset();
//            }
//        }
        if (gamepad2.dpad_up) {
            targetLiftHeight = 1;
        } else if (gamepad2.dpad_right) {
            targetLiftHeight = 2;
        } else if (gamepad2.dpad_down) {
            targetLiftHeight = 3;
        } else if (gamepad2.dpad_left) {
            targetLiftHeight = 4;
        } else if (gamepad2.y) {
            targetLiftHeight = 5;
        } else if (gamepad2.b) {
            targetLiftHeight = 6;
        }

        if (gamepad1.dpad_left) { // set lift mode to auto
            liftAutoMode = true;
            liftPID.setTarget(LIFT_STONE_POS[targetLiftHeight-1]);
        } else if (gamepad1.dpad_right) { // set lift mode to driver
            liftAutoMode = false;
            liftUpdatedPos = true;
        }

        if (gamepad1.y) { // auto lower lift and chainbar
            targetLiftHeight = 1;
            liftAutoMode = true;
            liftPID.setTarget(LIFT_STONE_POS[targetLiftHeight-1]);
            chainBarPID.setTarget(CHAINBAR_IN_VOLTAGE);
            chainBarPower = chainBarPID.getPIDOutput(chainBarPot.getVoltage());
            liftDown = true;
        }

        if (gamepad1.dpad_up) { // lift up with dampening
            liftPower = LIFT_DAMPEN_POWER*liftPower(lift.getCurrentPosition()/LIFT_MAX_POS);
            liftUpdatedPos = true;
            liftAutoMode = false;
        } else if (gamepad1.dpad_down) { // lift down
            liftPower = LIFT_RETRACTION_POWER;
            liftUpdatedPos = true;
            liftAutoMode = false;
        } else if (!liftAutoMode) { // lift hold
            if (liftUpdatedPos) { // lift update target pos
                liftUpdatedPos = false;
                liftPID.setTarget(lift.getCurrentPosition());
            }
            liftPower = LIFT_HOLD_POWER*liftPID.getPIDOutput(lift.getCurrentPosition());
        }

        if (liftAutoMode) { // auto raise
            double liftCV = liftPID.getPIDOutput(lift.getCurrentPosition());
            if (liftCV > 0) {
                lift.setPower(LIFT_EXTENSION_POWER*Math.abs(liftCV));
            } else if (liftCV < 0) {
                lift.setPower(LIFT_RETRACTION_POWER*Math.abs(liftCV));
            }
        } else { // manual raise
            lift.setPower(liftPower);
        }

        liftRaised = lift.getCurrentPosition() > LIFT_RAISED_MIN_POS;


        //-------------------------------------------- ChainBar --------------------------------------------

        if (gamepad1.left_bumper) { // chain bar in
            chainBarPower = -CHAINBAR_HIGH_POWER;
            liftDown = false;
            chainBarPID.setTarget(chainBarPot.getVoltage());
        } else if (gamepad1.right_bumper && chainBarPot.getVoltage() < CHAINBAR_MAX_VOLTAGE) { // chain bar out
            chainBarPower = CHAINBAR_HIGH_POWER;
            liftDown = false;
            chainBarPID.setTarget(chainBarPot.getVoltage());
        } else if (!intakeIn && !liftDown) {
            chainBarPower = chainBarPID.getPIDOutput(chainBarPot.getVoltage());
        }
        chainBar.setPower(chainBarPower);


        //-------------------------------------------- Claw --------------------------------------------

        if (gamepad1.a) { // claw close
            clawSquish.setPosition(CHAINBAR_CLAW_CLOSE_POS);
        } else if (gamepad1.b) { // claw open
            clawSquish.setPosition(CHAINBAR_CLAW_OPEN_POS);
        }


        //-------------------------------------------- Hook --------------------------------------------

        if (gamepad1.x && (toggleHook.milliseconds() > 250)) { // toggle foundation hook
            hookUp = !hookUp;
            if (hookUp) {
                foundationHookRight.setPosition(RIGHT_HOOK_UP_POS);
                foundationHookLeft.setPosition(LEFT_HOOK_UP_POS);
            } else {
                foundationHookRight.setPosition(RIGHT_HOOK_DOWN_POS);
                foundationHookLeft.setPosition(LEFT_HOOK_DOWN_POS);
            }
            toggleHook.reset();
        }


        //-------------------------------------------- Movement --------------------------------------------

        if (gamepad1.left_stick_button) { // toggle drive speed
            if (toggleDriveSpeed.milliseconds() > 250)  {
                drivePowerLow = !drivePowerLow;
                toggleDriveSpeed.reset();
            }
        }
        if (liftRaised || drivePowerLow) {
            drivePower = DRIVE_POWER_LOW;
        } else {
            drivePower = DRIVE_POWER_HIGH;
        }

        double strafeX = gamepad1.left_stick_x;
        double strafeY = -gamepad1.left_stick_y;
        double rotation = gamepad1.right_stick_x;
        double[] powerArray = drivePowerArray(strafeX, strafeY, rotation);

        lf.setPower(drivePower*powerArray[0]);
        lb.setPower(drivePower*powerArray[1]);
        rf.setPower(drivePower*powerArray[2]);
        rb.setPower(drivePower*powerArray[3]);


        //-------------------------------------------- Telemetry --------------------------------------------

        telemetry.addData("Drive Power",drivePower);
        telemetry.addData("Lift Mode",(liftAutoMode ? "auto" : "driver"));
        telemetry.addData("Lift Target Stone",targetLiftHeight);
        telemetry.addLine();
        telemetry.addData("Lift Raised",liftRaised);
        telemetry.addData("Lift Down",liftDown);
        telemetry.addData("Lift Current Position",lift.getCurrentPosition());
        telemetry.addData("Lift Target Position",liftPID.getTarget());
        telemetry.addData("Chain Bar Current Voltage",chainBarPot.getVoltage());
        telemetry.addData("Chain Bar Target Voltage",chainBarPID.getTarget());
        telemetry.addLine();
        telemetry.addData("Lift Power",liftPower);
        telemetry.addData("Chain Bar Power",chainBarPower);
        telemetry.addData("lf Power",lf.getPower());
        telemetry.addData("lb Power",lb.getPower());
        telemetry.addData("rf Power",rf.getPower());
        telemetry.addData("rb Power",rb.getPower());
        telemetry.addLine();
        telemetry.addData("foundationHookLeft Pos", foundationHookLeft.getPosition());
        telemetry.addData("foundationHookRight Pos", foundationHookRight.getPosition());
        telemetry.addData("clawSquish Pos", clawSquish.getPosition());

        telemetry.update();
    }

    private double liftPower(double ratio) {
        double L = 1;
        double b = .85;
        double k = 5.5;
        double v = 0.9;

        if (ratio < 0) {
            ratio = 0;
        }
        if (ratio >= 0 && ratio < v) {
            return L/(1+Math.pow(Math.E,k*(ratio-b)));
        } else if (ratio >= v && ratio <= 1) {
            return L/(1+Math.pow(Math.E, k*(v-b)));
        }
        return 0;
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