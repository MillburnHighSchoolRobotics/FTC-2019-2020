package org.firstinspires.ftc.teamcode.autonomous.general;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.MohanBot;


@Autonomous(group = "auton")
public class AutonParkStraightRight extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        MohanBot robot = new MohanBot(hardwareMap,this);
        waitForStart();

        if (isStopRequested()) return;

        robot.followTrajectory(
                robot.trajectoryBuilder()
                        .forward(24)
                        .strafeRight(24)
                        .build()
        );
    }
}
