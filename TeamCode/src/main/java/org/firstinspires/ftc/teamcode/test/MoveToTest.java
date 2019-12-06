package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.MohanBot;

@Autonomous(group = "drive")
public class MoveToTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        MohanBot drive = new MohanBot(hardwareMap,this,  new Pose2d());

        waitForStart();

        if (isStopRequested()) return;

//        drive.moveTo(new Vector2d(24 ,24), 0,0.8,0.3);

        drive.strafeTo(new Vector2d(24,24),0.8);
    }
}
