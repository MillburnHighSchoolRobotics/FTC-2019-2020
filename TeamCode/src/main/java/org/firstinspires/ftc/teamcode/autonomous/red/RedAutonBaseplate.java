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
    public Servo foundationHookLeft, foundationHookRight;
    final double[] foundationHookPos = {0,1};

    @Override
    public void runOpMode() throws InterruptedException {
        ThreadManager manager = ThreadManager.getInstance();
        manager.setHardwareMap(hardwareMap);
        manager.setCurrentAuton(this);
        manager.setupThread("PositionMonitor", PositionMonitor.class);

        DriveBase drive = new MohanBot(hardwareMap);
//        drive.setPoseEstimate(new Pose2d(24, 63, 0));

        foundationHookLeft = hardwareMap.servo.get("foundationHookLeft");
        foundationHookLeft.setPosition(foundationHookPos[0]);
        foundationHookRight = hardwareMap.servo.get("foundationHookRight");
        foundationHookRight.setPosition(foundationHookPos[0]);

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
                        .strafeRight(4)
                        .build()
        );
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .back(19)
                        .build()
        );
        foundationHookLeft.setPosition(foundationHookPos[1]);
        foundationHookRight.setPosition(foundationHookPos[1]);
        Thread.sleep(1000);
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .forward(48)
                        .build()
        );
        Thread.sleep(1000);
        foundationHookLeft.setPosition(foundationHookPos[0]);
        foundationHookRight.setPosition(foundationHookPos[0]);
        Thread.sleep(1000);
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .strafeRight(64)
                        .build()
        );

    }
}
