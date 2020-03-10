package com.millburnrobotics.skystone.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(group = "fuckerrooooooooof")
public class TimedAuton extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();

        DcMotor motor1 = hardwareMap.dcMotor.get("lf");
        DcMotor motor2 = hardwareMap.dcMotor.get("lb");
        DcMotor motor3 = hardwareMap.dcMotor.get("rf");
        DcMotor motor4 = hardwareMap.dcMotor.get("rb");

        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor4.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor4.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        motor1.setPower(0.4);
        motor2.setPower(0.4);
        motor3.setPower(-0.4);
        motor4.setPower(-0.4);
        Thread.sleep(500);
        motor1.setPower(0);
        motor2.setPower(0);
        motor3.setPower(0);
        motor4.setPower(0);

    }
}
