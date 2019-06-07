package org.firstinspires.ftc.teamcode.Tests;

import android.app.Activity;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.internal.opmode.OpModeManagerImpl;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Movement;

@Autonomous(name = "Encoder Test", group = "test")
public class EncoderTest extends LinearOpMode {

    DcMotor encoder;

    @Override
    public void runOpMode() throws InterruptedException {
        encoder = hardwareMap.dcMotor.get("motor");
        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        waitForStart();
        while (true) {
            Log.d("encoderPosition", String.valueOf(encoder.getCurrentPosition()));
        }
    }
}