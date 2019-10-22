package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.util.Movement;

@Autonomous(name = "Encoder Position Test", group = "test")
public class EncoderPositionTest extends LinearOpMode {

    DcMotorEx encoder;

    @Override
    public void runOpMode() throws InterruptedException {
        encoder = (DcMotorEx) hardwareMap.dcMotor.get("lb");
        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        waitForStart();
        while (true) {
            telemetry.addData("encoderPosition", String.valueOf(Movement.encoderToDistance(encoder.getCurrentPosition())));
            telemetry.update();
        }
    }
}