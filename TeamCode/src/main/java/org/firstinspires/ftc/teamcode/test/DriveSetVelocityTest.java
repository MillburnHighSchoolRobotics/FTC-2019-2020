package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.robot.GlobalConstants;

@TeleOp(group = "test")
public class DriveSetVelocityTest extends OpMode {
    private DcMotorEx lf;
    private double w_predicted = 0;
    private double startPosition = 0;

    @Override
    public void init() {
        lf = (DcMotorEx)hardwareMap.dcMotor.get("lf");
        lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lf.setPower(0);
    }

    @Override
    public void loop() {
        if (gamepad1.dpad_up) {
            w_predicted += 0.5;
        } else if (gamepad1.dpad_down) {
            w_predicted -= 0.5;
        }

        lf.setVelocity(w_predicted, AngleUnit.RADIANS);

        double w_calculated = (GlobalConstants.motorTicksToRadians(lf.getCurrentPosition() - startPosition))/(1);

        telemetry.addData("predicted w - ",w_predicted);
        telemetry.addData("calculated w - ",w_calculated);
        telemetry.update();
        startPosition = lf.getCurrentPosition();
        try {
            Thread.sleep(1000);
        } catch (InterruptedException ex) {
            ex.printStackTrace();
        }
    }
}