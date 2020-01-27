package com.millburnrobotics.skystone.auto.autons.blue;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.millburnrobotics.skystone.subsystems.MohanBot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.opencv.android.OpenCVLoader;


@Autonomous(group = "auton")
public class BlueAutonIntakeTwoBlock extends LinearOpMode {
    static {
        if (OpenCVLoader.initDebug()) {
            Log.d("opencv", "yay it works");
        } else {
            Log.d("opencv", "nope it doesnt work");
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        MohanBot robot = new MohanBot(hardwareMap, this, new Pose2d(-63,-39,3*Math.PI/2));
//        BarkerClass barker = new BarkerClass(hardwareMap, SIDE.BLUE);

        robot.hook.hookUp();
        robot.intake.intakeStop();
        robot.chainBar.openClaw();

//        telemetry.addData("Barker", "Waking...");
//        telemetry.update();
//
//        barker.wake();
//
//        telemetry.addData("Barker", "Waked!!!1!11!!");
//        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        int pos = 2;
        telemetry.addData("block",pos);
        telemetry.update();

        if (pos == 1) {
            robot.intake.intakeIn();
//            robot.chainBar.chainBarUp();
            robot.follow(0.1,0.7,
                    robot.path(3*Math.PI/2)
                            .splineTo(new Pose2d(new Vector2d(-34,-34).rotated(-Math.PI/2),Math.toRadians(315)))
//                            .splineTo(new Pose2d(new Vector2d(-27,-28).rotated(-Math.PI/2),Math.toRadians(350)))
                            .build(),
                    new double[]{317.5,325}
            );
//            Thread.sleep(250);
//            robot.follow(0.1,0.7,
//                    robot.path(Math.toRadians(95))
//                            .splineTo(new Pose2d(new Vector2d(-45,-15).rotated(-Math.PI/2),Math.toRadians(90)))
//                            .build(),
//                    new double[]{0}
//            );
//            robot.intake.intakeStop();
////            robot.chainBar.chainBarIn();
//            robot.chainBar.closeClaw();
        } else if (pos == 2) {
            robot.intake.intakeIn();
//            robot.chainBar.chainBarUp();
            robot.follow(0.1,0.7,
                    robot.path(3*Math.PI/2)
                            .splineTo(new Pose2d(new Vector2d(-36,-44.5).rotated(-Math.PI/2),Math.toRadians(300)))
                            .splineTo(new Pose2d(new Vector2d(-28,-41).rotated(-Math.PI/2),Math.toRadians(300)))
                            .build(),
                    new double[]{300,300}
            );
//            Thread.sleep(250);
//            robot.follow(0.1,0.7,
//                    robot.path(Math.toRadians(95))
//                            .splineTo(new Pose2d(new Vector2d(-45,-15).rotated(-Math.PI/2),Math.toRadians(90)))
//                            .build(),
//                    new double[]{0}
//            );
//            robot.intake.intakeStop();
////            robot.chainBar.chainBarIn();
//            robot.chainBar.closeClaw();
        } else {
            robot.intake.intakeIn();
//            robot.chainBar.chainBarUp();
            robot.follow(0.1,0.7,
                    robot.path(0)
                            .splineTo(new Pose2d(new Vector2d(-40,-32).rotated(-Math.PI/2),Math.toRadians(270)))
                            .splineTo(new Pose2d(new Vector2d(-22,-28).rotated(-Math.PI/2),Math.toRadians(330)))
                            .build(),
                    new double[]{180,180}
            );
            robot.strafeTo(new Vector2d(-22,-36),0.6);

            Thread.sleep(250);
            robot.follow(0.1,0.7,
                    robot.path(Math.toRadians(90))
                            .splineTo(new Pose2d(new Vector2d(-45,-15).rotated(-Math.PI/2),Math.toRadians(0)))
                            .build(),
                    new double[]{90}
            );
            robot.intake.intakeStop();
//            robot.chainBar.chainBarIn();
//            robot.chainBar.closeClaw();
        }
//        robot.follow(0.1,0.7,
//                robot.path(0)
//                        .splineTo(new Pose2d(new Vector2d(-41,42).rotated(-Math.PI/2),0))
//                        .build(),
//                new double[]{90}
//        );
//        robot.strafeTo(new Vector2d(-25,42),0.4);
//        robot.hook.hookDown();
//        Thread.sleep(500);
//        robot.follow(0.3,0.9,
//                robot.path(3*Math.PI/2)
//                        .splineTo(new Pose2d(new Vector2d(-48,24).rotated(-Math.PI/2),Math.PI))
//                        .build(),
//                new double[]{180}
//        );
//        robot.chainBar.chainBarOut();
//        robot.chainBar.openClaw();
//        robot.hook.hookUp();
//        robot.strafeTo(new Vector2d(-48,36),0.6);
//        robot.chainBar.chainBarIn();
//        robot.strafeTo(new Vector2d(-40,-12),0.7);
//
//        robot.chainBar.chainBarUp();
//        robot.intake.intakeIn();
//        robot.follow(0.1,0.7,
//                robot.path(Math.PI)
//                        .splineTo(new Pose2d(new Vector2d(-40,-40).rotated(-Math.PI/2),Math.toRadians(205)))
//                        .splineTo(new Pose2d(new Vector2d(-35,-45.5).rotated(-Math.PI/2),Math.toRadians(190)))
//                        .build(),
//                new double[]{205,190}
//        );
    }
}
