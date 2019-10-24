package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(group = "test")
public class ChainBarPositionTest extends OpMode {
    final double chainBarPower = 0.8;

    public DcMotor chainBar;
    @Override
    public void init() {
        chainBar = hardwareMap.dcMotor.get("chainBar");
        chainBar.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        chainBar.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        chainBar.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        chainBar.setPower(0);
    }

    @Override
    public void loop() {

        if (gamepad1.left_bumper) {
            chainBar.setPower(-chainBarPower);
        } else if (gamepad1.right_bumper) {
            chainBar.setPower(chainBarPower);
        } else {
            chainBar.setPower(0);
        }
        telemetry.addData("ChainBar",chainBar.getCurrentPosition());
        telemetry.update();
    }
}