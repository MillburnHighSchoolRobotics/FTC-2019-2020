package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.MohanBot;

@Autonomous(group = "test")
public class SpamRotate extends LinearOpMode {

    public void runOpMode() throws InterruptedException {
        MohanBot robot = new MohanBot(hardwareMap,this, new Pose2d(0,0,0));

        waitForStart();

        if (isStopRequested()) return;

        while(true) {
            robot.rotate(90,1);
            Thread.sleep(1000);

            telemetry.addData("heading",""+robot.getPose().getHeading());
            telemetry.update();
        }
    }
}
