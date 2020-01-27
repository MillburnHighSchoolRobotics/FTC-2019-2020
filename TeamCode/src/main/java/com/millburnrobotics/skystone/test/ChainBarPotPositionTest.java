package com.millburnrobotics.skystone.test;

import com.millburnrobotics.skystone.subsystems.MohanBot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(group = "test")
public class ChainBarPotPositionTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        MohanBot robot = new MohanBot(hardwareMap, this);

        waitForStart();

        if (isStopRequested()) return;

        robot.chainBar.chainBarIn();
        Thread.sleep(2000);

        robot.chainBar.chainBarUp();
        Thread.sleep(2000);

        robot.chainBar.chainBarOut();
        Thread.sleep(2000);

        robot.chainBar.chainBarIn();
        robot.intake.intakeIn();
        Thread.sleep(2000);
        robot.intake.intakeStop();

        robot.chainBar.chainBarOut();
        robot.intake.intakeIn();
        Thread.sleep(2000);
        robot.intake.intakeStop();
    }

}
