package org.firstinspires.ftc.teamcode.autonomous.blue;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.heading.ConstantInterpolator;
import com.acmerobotics.roadrunner.path.heading.SplineInterpolator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.MohanBot;

@Autonomous(group = "auton")
public class BlueAutonEverything extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        MohanBot mohanBot = new MohanBot(hardwareMap,this);
        mohanBot.setPose(new Pose2d(-39,63,3*Math.PI/2));

        mohanBot.getHook().hookUp();
        mohanBot.getChainBar().openClaw();

        waitForStart();
        if (isStopRequested()) return;

        mohanBot.getChainBar().chainBarUp();
        mohanBot.getIntake().intakeIn();

        mohanBot.followTrajectory(
                mohanBot.trajectoryBuilder()
                        .strafeTo(new Vector2d(-36,55.5))
                        .build()
        );

        int skystone = 2;//(int)Math.round(3*Math.random())+1;
        switch(skystone) {
            case 1:
                mohanBot.followTrajectory(
                        mohanBot.trajectoryBuilder()
                                .splineTo(new Pose2d(-44,28,Math.toRadians(225)))
                                .strafeTo(new Vector2d(-46,24))
                                .build()
                );
                break;
            case 2:
                mohanBot.followTrajectory(
                        mohanBot.trajectoryBuilder()
                                .splineTo(new Pose2d(-52,28,Math.toRadians(225)))
                                .strafeTo(new Vector2d(-54,24))
                                .build()
                );
                break;
            case 3:
                mohanBot.followTrajectory(
                        mohanBot.trajectoryBuilder()
                                .splineTo(new Pose2d(-60,28,Math.toRadians(225)))
                                .strafeTo(new Vector2d(-62,24))
                                .build()
                );
                break;

        }
        mohanBot.getChainBar().chainBarIn();
        mohanBot.getChainBar().closeClaw();

        mohanBot.followTrajectory(
                mohanBot.trajectoryBuilder()
                        .reverse()
                        .splineTo(new Pose2d(24,40,Math.PI))
                        .splineTo(new Pose2d(51,24,Math.PI/2), new SplineInterpolator(Math.PI/2,Math.PI))
                        .strafeTo(new Vector2d(51,20))
                        .build()
        );
        mohanBot.getIntake().intakeStop();
        mohanBot.getHook().hookDown();
        mohanBot.getChainBar().chainBarOut();

        mohanBot.followTrajectory(
                mohanBot.trajectoryBuilder()
                        .setReversed(false)
                        .splineTo(new Pose2d(24,48,Math.PI))
                        .build()
        );

        mohanBot.getHook().hookUp();
        mohanBot.getChainBar().chainBarIn();
        mohanBot.getIntake().intakeIn();

        mohanBot.followTrajectory(
                mohanBot.trajectoryBuilder()
                        .splineTo(new Pose2d(0,42,Math.PI),new ConstantInterpolator(Math.PI))
                        .build()
        );
        mohanBot.getChainBar().chainBarUp();

        switch(skystone) {
            case 1:
                mohanBot.followTrajectory(
                        mohanBot.trajectoryBuilder()
                                .splineTo(new Pose2d(-24,32,3*Math.PI/2), new SplineInterpolator(Math.PI,Math.toRadians(225)))
                                .build()
                );
                break;
            case 2:
                mohanBot.followTrajectory(
                        mohanBot.trajectoryBuilder()
                                .splineTo(new Pose2d(-32,32,3*Math.PI/2), new SplineInterpolator(Math.PI,Math.toRadians(225)))
                                .build()
                );
                break;
            case 3:
                mohanBot.followTrajectory(
                        mohanBot.trajectoryBuilder()
                                .splineTo(new Pose2d(-40,32,3*Math.PI/2), new SplineInterpolator(Math.PI,Math.toRadians(225)))
                                .build()
                );
                break;
        }

        mohanBot.getChainBar().chainBarIn();
        mohanBot.getChainBar().closeClaw();

        mohanBot.followTrajectory(
                mohanBot.trajectoryBuilder()
                        .reverse()
                        .splineTo(new Pose2d(24,48,Math.PI))
                        .build()
        );

        mohanBot.getIntake().intakeStop();
        mohanBot.getChainBar().chainBarOut();
        mohanBot.getChainBar().openClaw();

        mohanBot.followTrajectory(
                mohanBot.trajectoryBuilder()
                        .reverse()
                        .back(48)
                        .build()
        );

        mohanBot.getChainBar().chainBarIn();
        mohanBot.followTrajectory(
                mohanBot.trajectoryBuilder()
                        .splineTo(new Pose2d(0,42,Math.PI), new ConstantInterpolator(Math.PI))
                        .build()
        );
    }
}
