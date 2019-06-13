package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Autonomous(name = "Encoder Position Test", group = "test")
public class EncoderPositionTest extends LinearOpMode {

    DcMotorEx encoder;

    @Override
    public void runOpMode() throws InterruptedException {
        encoder = (DcMotorEx) hardwareMap.dcMotor.get("lf");
        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        waitForStart();
        while (true) {
            telemetry.addData("encoderPosition", String.valueOf(encoder.getCurrentPosition()));
            telemetry.update();
        }
    }
}