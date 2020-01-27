package com.millburnrobotics.skystone.auto.autons.general;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.millburnrobotics.skystone.subsystems.MohanBot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


@Autonomous(group = "auton")
public class AutonParkStraight extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        MohanBot robot = new MohanBot(hardwareMap,this);
        waitForStart();

        if (isStopRequested()) return;

        robot.strafeTo(new Vector2d(0,24),0.7);
    }
}
