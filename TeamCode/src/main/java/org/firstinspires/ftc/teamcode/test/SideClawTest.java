package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.MohanBot;

@Autonomous(group = "test")
public class SideClawTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        MohanBot robot = new MohanBot(hardwareMap, this);

        waitForStart();

        if (isStopRequested()) return;

        robot.sideClaw.barUp();
        Thread.sleep(2000);

        robot.sideClaw.barDown();
        Thread.sleep(2000);

        robot.sideClaw.openClaw();
        Thread.sleep(2000);

        robot.sideClaw.closeClaw();
        Thread.sleep(2000);

        robot.sideClaw.hideClaw();
        Thread.sleep(2000);
    }

}
