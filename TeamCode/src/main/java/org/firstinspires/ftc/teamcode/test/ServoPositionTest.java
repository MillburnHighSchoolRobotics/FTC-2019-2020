package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(group = "test")
public class ServoPositionTest extends OpMode {

    public Servo servo;
    @Override
    public void init() {
        servo = hardwareMap.servo.get("foundationHook");
        servo.setPosition(0);
    }

    @Override
    public void loop() {
        for (int x = 0; x <= 10; x++) {
            servo.setPosition(x/10.0);
            telemetry.addData("Servo",servo.getPosition());
            telemetry.update();
            try {
                Thread.sleep(2000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }
}