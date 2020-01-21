package com.millburnrobotics.skystone.autonomous.red;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.millburnrobotics.skystone.Constants;
import com.millburnrobotics.skystone.robot.MohanBot;
import com.millburnrobotics.skystone.util.BarkerClass;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.opencv.android.OpenCVLoader;


@Autonomous(group = "auton")
public class RedAutonThreeBlock extends LinearOpMode {
    static {
        if (OpenCVLoader.initDebug()) {
            Log.d("opencv", "yay it works");
        } else {
            Log.d("opencv", "nope it doesnt work");
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        MohanBot robot = new MohanBot(hardwareMap, this, new Pose2d(63,-39,Math.PI/2));
        BarkerClass barker = new BarkerClass(hardwareMap, Constants.SIDE.RED);

        robot.hook.hookUp();
        robot.intake.intakeStop();
        robot.chainBar.closeClaw();
        robot.sideClaw.initClaw();
        robot.sideClaw.barUp();

        telemetry.addData("position", robot.getPose().toString());
        telemetry.update();

        telemetry.addData("Barker", "Waking...");
        telemetry.update();

        barker.wake();

        telemetry.addData("Barker", "Waked!!!1!11!!");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        int pos = barker.bark();
        telemetry.addData("block",pos);
        telemetry.update();

        if (pos == 3) {
            robot.sideClaw.barDown();
            robot.follow(0.1,0.7,
                    robot.path(Math.toRadians(165))
                            .splineTo(new Pose2d(Constants.AutonConstants.RED_BLOCK_4.rotated(-Math.PI/2),Math.toRadians(90)))
                            .build(),
                    new double[]{0}, true, 6
            );
            robot.follow(0.1,0.5,
                    robot.path(0)
                            .strafeLeft(4)
                            .build(),
                    new double[]{0}, false, 6
            );
            robot.sideClaw.closeClaw();
            Thread.sleep(500);
            robot.sideClaw.barMid();
            Thread.sleep(500);

            robot.follow(0.2,0.7,
                    robot.path(3*Math.PI/2)
                            .lineTo(new Vector2d(39,-40).rotated(-Math.PI/2)) // 72-18-9-3
                            .splineTo(new Pose2d(new Vector2d(40,-24).rotated(-Math.PI/2),Math.toRadians(0))) // 72-18-9-3
                            .splineTo(new Pose2d(new Vector2d(40,36).rotated(-Math.PI/2),Math.toRadians(0)))
                            .splineTo(new Pose2d(new Vector2d(29,51).rotated(-Math.PI/2),Math.toRadians(60)))
                            .build(),
                    new double[]{270,270,270,0}
            );
        } else if (pos == 2) {
            robot.sideClaw.barDown();
            robot.follow(0.1,0.7,
                    robot.path(Math.PI)
                            .splineTo(new Pose2d(Constants.AutonConstants.RED_BLOCK_2.rotated(-Math.PI/2),Math.toRadians(115)))
                            .build(),
                    new double[]{0}, true, 6
            );
            robot.follow(0.1,0.5,
                    robot.path(0)
                            .strafeLeft(4)
                            .build(),
                    new double[]{0}, false, 6
            );
            robot.sideClaw.closeClaw();
            Thread.sleep(500);
            robot.sideClaw.barMid();
            Thread.sleep(500);

            robot.follow(0.2,0.7,
                    robot.path(3*Math.PI/2)
                            .lineTo(new Vector2d(39,-56).rotated(-Math.PI/2)) // 72-18-9-3
                            .splineTo(new Pose2d(new Vector2d(40,-24).rotated(-Math.PI/2),Math.toRadians(0))) // 72-18-9-3
                            .splineTo(new Pose2d(new Vector2d(40,36).rotated(-Math.PI/2),Math.toRadians(0)))
                            .splineTo(new Pose2d(new Vector2d(29,44).rotated(-Math.PI/2),Math.toRadians(60)))
                            .build(),
                    new double[]{270,270,270,0}
            );
        } else {
            robot.sideClaw.barDown();
            robot.follow(0.1,0.7,
                    robot.path(Math.PI)
                            .splineTo(new Pose2d(Constants.AutonConstants.RED_BLOCK_3.rotated(-Math.PI/2),Math.toRadians(65)))
                            .build(),
                    new double[]{0}, true, 6
            );
            robot.follow(0.1,0.5,
                    robot.path(0)
                            .strafeLeft(4)
                            .build(),
                    new double[]{0}, false, 6
            );
            robot.sideClaw.closeClaw();
            Thread.sleep(500);
            robot.sideClaw.barMid();
            Thread.sleep(500);

            robot.follow(0.2,0.7,
                    robot.path(3*Math.PI/2)
                            .lineTo(new Vector2d(39,-48).rotated(-Math.PI/2)) // 72-18-9-3
                            .splineTo(new Pose2d(new Vector2d(40,-24).rotated(-Math.PI/2),Math.toRadians(0))) // 72-18-9-3
                            .splineTo(new Pose2d(new Vector2d(40,36).rotated(-Math.PI/2),Math.toRadians(0)))
                            .splineTo(new Pose2d(new Vector2d(29,44).rotated(-Math.PI/2),Math.toRadians(60)))
                            .build(),
                    new double[]{270,270,270,0}
            );
        }

        robot.sideClaw.barDown();
        Thread.sleep(100);
        robot.sideClaw.openClaw();
        Thread.sleep(1000);


        if (pos == 3) {
            robot.sideClaw.barUp();
            robot.sideClaw.openClaw();

            robot.intake.intakeIn();
            robot.follow(0.25,0.7,
                    robot.path(3*Math.PI/2)
                            .splineTo(new Pose2d(new Vector2d(40,36).rotated(-Math.PI/2),Math.toRadians(180)))
                            .splineTo(new Pose2d(new Vector2d(40,-24).rotated(-Math.PI/2),Math.toRadians(180)))
                            .splineTo(new Pose2d(new Vector2d(24,-48-8+2).rotated(-Math.PI/2),Math.toRadians(90)))
                            .build(),
                    new double[]{180,180,180}
            );
            robot.follow(0.3,0.5,
                    robot.path(Math.PI)
                            .forward(8)
                            .build(),
                    new double[]{180},false
            );
        } else if (pos == 2) {
            robot.sideClaw.barDown();
            robot.sideClaw.openClaw();
            robot.follow(0.1,0.9,
                    robot.path(3*Math.PI/2)
                            .splineTo(new Pose2d(new Vector2d(36,24).rotated(-Math.PI/2),Math.toRadians(180)))
                            .splineTo(new Pose2d(new Vector2d(42,-20).rotated(-Math.PI/2),Math.toRadians(180)))
                            .splineTo(new Pose2d(Constants.AutonConstants.RED_BLOCK_5.rotated(-Math.PI/2),Math.toRadians(160)))
                            .build(),
                    new double[]{0,0,0}, true, 6
            );
            robot.follow(0.1,0.5,
                    robot.path(0)
                            .strafeLeft(4)
                            .build(),
                    new double[]{0}, false, 6
            );

            robot.sideClaw.closeClaw();
            Thread.sleep(1000);
            robot.sideClaw.barMid();
            Thread.sleep(500);
        } else {
            robot.sideClaw.barDown();
            robot.sideClaw.openClaw();
            robot.follow(0.1,0.9,
                    robot.path(3*Math.PI/2)
                            .splineTo(new Pose2d(new Vector2d(36,24).rotated(-Math.PI/2),Math.toRadians(180)))
                            .splineTo(new Pose2d(new Vector2d(40,-12).rotated(-Math.PI/2),Math.toRadians(180)))
                            .splineTo(new Pose2d(Constants.AutonConstants.RED_BLOCK_6.rotated(-Math.PI/2),Math.toRadians(160)))
                            .build(),
                    new double[]{0,0,0}, true, 6
            );
            robot.follow(0.1,0.5,
                    robot.path(0)
                            .strafeLeft(4)
                            .build(),
                    new double[]{0}, false, 6
            );
            robot.sideClaw.closeClaw();
            Thread.sleep(1000);
            robot.sideClaw.barMid();
            Thread.sleep(500);
        }

        if(pos == 1 || pos == 2) {
            robot.follow(0.2,0.7,
                    robot.path(3*Math.PI/2)
                            .lineTo(new Vector2d(-37,-32).rotated(-Math.PI/2)) // 72-18-9-3
                            .splineTo(new Pose2d(new Vector2d(40,-24).rotated(-Math.PI/2),Math.toRadians(0))) // 72-18-9-3
                            .splineTo(new Pose2d(new Vector2d(40,36).rotated(-Math.PI/2),Math.toRadians(0)))
                            .splineTo(new Pose2d(new Vector2d(31,52).rotated(-Math.PI/2),Math.toRadians(90)))
                            .build(),
                    new double[]{270,270,270,0}
            );

            robot.sideClaw.barDown();
            Thread.sleep(100);
            robot.sideClaw.openClaw();
            Thread.sleep(1000);
            robot.rotateTo(270, 1);
            robot.follow(0.1,0.5,
                    robot.path(3*Math.PI/2)
                            .back(6)
                            .build(),
                    new double[]{270}
            );

            robot.hook.hookDown();
            Thread.sleep(1000);
        } else {
            robot.follow(0.2,0.9,
                    robot.path(3*Math.PI/2)
                            .splineTo(new Pose2d(new Vector2d(40,-24).rotated(-Math.PI/2),Math.toRadians(0))) // 72-18-9-3
                            .splineTo(new Pose2d(new Vector2d(40,20).rotated(-Math.PI/2),Math.toRadians(0)))
                            .build(),
                    new double[]{180,90}
            );
            robot.intake.intakeOut();
            Thread.sleep(1000);
            robot.rotateTo(270,1);
            robot.follow(0.1,0.8,
                    robot.path(0)
                            .splineTo(new Pose2d(new Vector2d(40,31).rotated(-Math.PI/2),Math.toRadians(0)))
                            .splineTo(new Pose2d(new Vector2d(31,46).rotated(-Math.PI/2),Math.toRadians(90)))
                            .splineTo(new Pose2d(new Vector2d(28,46).rotated(-Math.PI/2),Math.toRadians(90)))
                            .build(),
                    new double[]{270,270,270}
            );
            robot.hook.hookDown();
            Thread.sleep(1000);
        }
        ElapsedTime gayshit = new ElapsedTime();
        while (gayshit.milliseconds() < 1500) {
            robot.drive.setDrivePower(1);
        }
        robot.rotateTo(180, 1, 5);
        robot.hook.hookUp();
        robot.strafeTo(new Vector2d(40,0),1);

    }
}
