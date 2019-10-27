package org.firstinspires.ftc.teamcode.autonomous.red;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.mecanum.DriveBase;
import org.firstinspires.ftc.teamcode.drive.mecanum.MohanBot;
import org.firstinspires.ftc.teamcode.threads.PositionMonitor;
import org.firstinspires.ftc.teamcode.threads.ThreadManager;


@Autonomous(group = "auton")
public class RedAutonBaseplate extends LinearOpMode {
    public Servo foundationHook;
    final double[] foundationHookPos = {0,1};

    @Override
    public void runOpMode() throws InterruptedException {
        ThreadManager manager = ThreadManager.getInstance();
        manager.setHardwareMap(hardwareMap);
        manager.setCurrentAuton(this);
        manager.setupThread("PositionMonitor", PositionMonitor.class);

        DriveBase drive = new MohanBot(hardwareMap);
        drive.setPoseEstimate(new Pose2d(24, -63, 0));

        foundationHook = hardwareMap.servo.get("foundationHook");
        foundationHook.setPosition(foundationHookPos[0]);

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .strafeRight(24)
                        .forward(15)
                        .build()
        );
        drive.turnSync(Math.PI);

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .strafeLeft(24)
                        .build()
        );
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .strafeRight(16)
                        .build()
        );
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .back(24)
                        .build()
        );
        foundationHook.setPosition(foundationHookPos[1]);
        Thread.sleep(1000);
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .forward(53)
                        .build()
        );
        Thread.sleep(1000);
        foundationHook.setPosition(foundationHookPos[0]);
        Thread.sleep(1000);
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .strafeRight(30)
                        .build()
        );
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .back(16)
                        .build()
        );
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .strafeLeft(5)
                        .build()
        );
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .strafeRight(27)
                        .build()
        );

    }
}
