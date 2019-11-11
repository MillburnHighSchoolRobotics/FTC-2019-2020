package org.firstinspires.ftc.teamcode.roadrunner;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.MohanBot;
import org.firstinspires.ftc.teamcode.threads.PositionMonitor;
import org.firstinspires.ftc.teamcode.threads.ThreadManager;


@Autonomous(group = "drive")
public class FollowerPIDTuner extends LinearOpMode {
    public static double DISTANCE = 48;

    @Override
    public void runOpMode() throws InterruptedException {
        ThreadManager manager = ThreadManager.getInstance();
        manager.setHardwareMap(hardwareMap);
        manager.setCurrentAuton(this);
        manager.setupThread("PositionMonitor", PositionMonitor.class);

        MohanBot robot = new MohanBot(hardwareMap,this);

        waitForStart();

        if (isStopRequested()) return;

        while (!isStopRequested()) {
            robot.followTrajectory(
                    robot.trajectoryBuilder()
                            .strafeTo(new Vector2d(DISTANCE,0))
                            .build()
            );
            robot.turn(Math.toRadians(180));
            Thread.sleep(1000);
        }
    }
}
