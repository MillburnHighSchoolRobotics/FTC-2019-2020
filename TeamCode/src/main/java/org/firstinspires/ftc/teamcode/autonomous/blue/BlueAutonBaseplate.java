package org.firstinspires.ftc.teamcode.autonomous.blue;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.path.heading.SplineInterpolator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.mecanum.DriveBase;
import org.firstinspires.ftc.teamcode.drive.mecanum.MohanBot;
import org.firstinspires.ftc.teamcode.threads.PositionMonitor;
import org.firstinspires.ftc.teamcode.threads.ThreadManager;


@Autonomous(group = "auton")
public class BlueAutonBaseplate extends LinearOpMode {
    public Servo foundationHookLeft, foundationHookRight;
    final double[] foundationHookPos = {1,0};

    @Override
    public void runOpMode() throws InterruptedException {
        ThreadManager manager = ThreadManager.getInstance();
        manager.setHardwareMap(hardwareMap);
        manager.setCurrentAuton(this);
        manager.setupThread("PositionMonitor", PositionMonitor.class);

        DriveBase drive = new MohanBot(hardwareMap);

        foundationHookLeft = hardwareMap.servo.get("foundationHookLeft");
        foundationHookLeft.setPosition(foundationHookPos[1]);
        foundationHookRight = hardwareMap.servo.get("foundationHookRight");
        foundationHookRight.setPosition(foundationHookPos[0]);

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .splineTo(new Pose2d(26,28,0), new SplineInterpolator(0,Math.PI))
                        .back(4)
                        .build()
        );
        foundationHookLeft.setPosition(foundationHookPos[0]);
        foundationHookRight.setPosition(foundationHookPos[1]);
        Thread.sleep(1000);
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .forward(48)
                        .build()
        );
        Thread.sleep(1000);
        foundationHookLeft.setPosition(foundationHookPos[1]);
        foundationHookRight.setPosition(foundationHookPos[0]);
        Thread.sleep(1000);
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .strafeLeft(69)
                        .build()
        );

    }
}
