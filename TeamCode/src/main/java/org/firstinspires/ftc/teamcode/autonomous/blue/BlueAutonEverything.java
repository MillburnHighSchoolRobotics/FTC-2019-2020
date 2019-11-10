package org.firstinspires.ftc.teamcode.autonomous.blue;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.heading.ConstantInterpolator;
import com.acmerobotics.roadrunner.path.heading.SplineInterpolator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.MillburnRobot;

@Autonomous(group = "auton")
public class BlueAutonEverything extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        MillburnRobot mohanBot = new MillburnRobot(hardwareMap,this);
        mohanBot.setPose(new Pose2d(-39,63,3*Math.PI/2));

        mohanBot.hookUp();
        mohanBot.openClaw();

        waitForStart();
        if (isStopRequested()) return;


        mohanBot.chainBarUp();
        mohanBot.intakeIn();

        mohanBot.followTrajectory(
                mohanBot.trajectoryBuilder()
                        .lineTo(new Vector2d(-39,57))
                .build()
        );

        int skystone = 2;//(int)Math.round(3*Math.random())+1;
        switch(skystone) {
            case 1:
                mohanBot.followTrajectory(
                        mohanBot.trajectoryBuilder()
                                .splineTo(new Pose2d(-48,32,Math.toRadians(225)))
                                .build()
                );
                break;
            case 2:
                mohanBot.followTrajectory(
                        mohanBot.trajectoryBuilder()
                                .splineTo(new Pose2d(-56,32,Math.toRadians(225)))
                                .build()
                );
                break;
            case 3:
                mohanBot.followTrajectory(
                        mohanBot.trajectoryBuilder()
                                .splineTo(new Pose2d(-64,32,Math.toRadians(225)))
                                .build()
                );
                break;

        }
        mohanBot.chainBarIn();
        mohanBot.closeClaw();

        mohanBot.followTrajectory(
                mohanBot.trajectoryBuilder()
                        .reverse()
                        .splineTo(new Pose2d(24,40,Math.PI))
                        .splineTo(new Pose2d(51,24,Math.PI/2), new SplineInterpolator(Math.PI,Math.PI/2))
                        .strafeTo(new Vector2d(51,20))
                        .build()
        );
        mohanBot.intakeStop();
        mohanBot.hookDown();

        mohanBot.chainBarOut();

        mohanBot.followTrajectory(
                mohanBot.trajectoryBuilder()
                        .setReversed(false)
                        .splineTo(new Pose2d(24,48,Math.PI))
                        .build()
        );

        mohanBot.hookUp();
        mohanBot.chainBarIn();
        mohanBot.intakeIn();

        mohanBot.followTrajectory(
                mohanBot.trajectoryBuilder()
                        .splineTo(new Pose2d(0,42,Math.PI),new ConstantInterpolator(Math.PI))
                        .build()
        );
        mohanBot.chainBarUp();

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
//                                .strafeTo(new Vector2d(-50,20))
                                .build()
                );
                break;
            case 3:
                mohanBot.followTrajectory(
                        mohanBot.trajectoryBuilder()
                                .splineTo(new Pose2d(-40,32,3*Math.PI/2), new SplineInterpolator(Math.PI,Math.toRadians(225)))
//                                .strafeTo(new Vector2d(-58,20))
                                .build()
                );
                break;

        }

        mohanBot.chainBarIn();
        mohanBot.closeClaw();

        mohanBot.followTrajectory(
                mohanBot.trajectoryBuilder()
                        .reverse()
                        .splineTo(new Pose2d(24,48,Math.PI))
                        .build()
        );
        mohanBot.intakeStop();
        mohanBot.chainBarOut();
        mohanBot.openClaw();

        mohanBot.followTrajectory(
                mohanBot.trajectoryBuilder()
                        .reverse()
                        .back(48)
                        .build()
        );

        mohanBot.chainBarIn();
        mohanBot.followTrajectory(
                mohanBot.trajectoryBuilder()
                        .splineTo(new Pose2d(0,42,Math.PI), new ConstantInterpolator(Math.PI))
                        .build()
        );
    }
}
