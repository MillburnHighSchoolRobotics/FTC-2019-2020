package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.MohanBot;

import java.nio.charset.IllegalCharsetNameException;

public class SpamRotate extends LinearOpMode {
    public void runOpMode() throws InterruptedException {
        MohanBot robot = new MohanBot(hardwareMap,this, new Pose2d(24, 63, Math.toRadians(90)));

        waitForStart();

        if (isStopRequested()) return;

        while(true) {
            robot.rotate(90);
            Thread.sleep(1000);
        }
    }
}
