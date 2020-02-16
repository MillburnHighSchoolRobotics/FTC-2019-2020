package com.millburnrobotics.skystone.test;

import com.millburnrobotics.skystone.subsystems.Robot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(group = "test")
public class LiftTest extends OpMode {
    @Override
    public void init() {
        telemetry.setMsTransmissionInterval(1000);
        Robot.getInstance().init(hardwareMap, false);
    }

    @Override
    public void loop() {
        if (gamepad1.dpad_up) {
            Robot.getInstance().getLift().setLiftPower(0.6);
        } else if (gamepad1.dpad_down) {
            Robot.getInstance().getLift().setLiftPower(-0.6);
        } else {
            Robot.getInstance().getLift().setLiftPower(0);
        }

        Robot.getInstance().outputToTelemetry(telemetry);
        telemetry.update();
    }
}
