package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.teamcode.robot.MohanBot;

@Autonomous(group = "util")
public class SpamRotate extends LinearOpMode {

    public void runOpMode() throws InterruptedException {
        MohanBot robot = new MohanBot(hardwareMap,this);

        double p = robot.getTurnPID().p;
        double i = robot.getTurnPID().i;
        double d = robot.getTurnPID().d;

        waitForStart();

        if (isStopRequested()) return;

        while(true) {
            if (gamepad1.dpad_up) {
                p += 0.001;
            } else if (gamepad1.dpad_down) {
                p -= 0.001;
            }
            if (gamepad1.a) {
                i += 0.001;
            } else if (gamepad1.b) {
                i -= 0.001;
            }
            if (gamepad1.x) {
                d += 0.001;
            } else if (gamepad1.y) {
                d -= 0.001;
            }

            robot.setTurnPID(new PIDCoefficients(p,i,d));

            telemetry.addData("p", p+"");
            telemetry.addData("i", i+"");
            telemetry.addData("d", d+"");
            telemetry.addData("heading",""+Math.toDegrees(robot.getPose().getHeading()));
            telemetry.update();

            robot.rotate(90,1);
            Thread.sleep(1000);
        }
    }
}
