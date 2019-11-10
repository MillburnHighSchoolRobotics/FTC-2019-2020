package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.MathUtils;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

@TeleOp(group = "test")
public class IWantTheBotIn18 extends OpMode {
    public DcMotorEx lf;
    public DcMotorEx lb;
    public DcMotorEx rf;
    public DcMotorEx rb;

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
    }

    @Override
    public void loop() {

        if (gamepad2.a) {
            lf.setPower(-1);
            rf.setPower(-1);
            lb.setPower(1);
            rb.setPower(1);
        } else {
            lf.setPower(0);
            lb.setPower(0);
            rf.setPower(0);
            rb.setPower(0);

        }
    }
}