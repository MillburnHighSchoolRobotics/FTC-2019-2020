package org.firstinspires.ftc.teamcode.autonomous.blue;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.ftccommon.configuration.EditLegacyModuleControllerActivity;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.GlobalConstants;
import org.firstinspires.ftc.teamcode.robot.MohanBot;
import org.firstinspires.ftc.teamcode.util.BarkerClass;
import org.opencv.android.OpenCVLoader;
import static org.firstinspires.ftc.teamcode.robot.GlobalConstants.*;


@Autonomous(group = "auton")
public class BlueAutonThreeBlock extends LinearOpMode {
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
        BarkerClass barker = new BarkerClass(hardwareMap, SIDE.BLUE);

        robot.hook.hookUp();
        robot.intake.intakeStop();
        robot.chainBar.closeClaw();
        robot.sideClaw.openClaw();
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
            robot.sideClaw.openClaw();
            robot.follow(0.1,0.7,
                    robot.path(Math.PI)
                            .splineTo(new Pose2d(GlobalConstants.BLUE_BLOCK_4.rotated(-Math.PI/2),Math.toRadians(245)))
                            .build(),
                    new double[]{180}, true, 6
            );
            robot.sideClaw.closeClaw();
            Thread.sleep(500);
            robot.sideClaw.barMid();

            robot.follow(0.2,0.7,
                    robot.path(Math.PI/2)
                            .lineTo(new Vector2d(-39,-48).rotated(-Math.PI/2)) // 72-18-9-3
                            .splineTo(new Pose2d(new Vector2d(-40,-24).rotated(-Math.PI/2),Math.toRadians(0))) // 72-18-9-3
                            .splineTo(new Pose2d(new Vector2d(-40,36).rotated(-Math.PI/2),Math.toRadians(0)))
                            .splineTo(new Pose2d(new Vector2d(-31,51).rotated(-Math.PI/2),Math.toRadians(300)))
                            .build(),
                    new double[]{90,90,90,180}
            );
        } else if (pos == 2) {
            robot.sideClaw.barDown();
            robot.sideClaw.openClaw();
            robot.follow(0.1,0.7,
                    robot.path(Math.PI)
                            .splineTo(new Pose2d(GlobalConstants.BLUE_BLOCK_5.rotated(-Math.PI/2),Math.toRadians(245)))
                            .build(),
                    new double[]{180}, true, 6
            );
            robot.sideClaw.closeClaw();
            Thread.sleep(500);
            robot.sideClaw.barMid();

            robot.follow(0.2,0.7,
                    robot.path(Math.PI/2)
                            .lineTo(new Vector2d(-39,-40).rotated(-Math.PI/2)) // 72-18-9-3
                            .splineTo(new Pose2d(new Vector2d(-40,-24).rotated(-Math.PI/2),Math.toRadians(0))) // 72-18-9-3
                            .splineTo(new Pose2d(new Vector2d(-40,36).rotated(-Math.PI/2),Math.toRadians(0)))
                            .splineTo(new Pose2d(new Vector2d(-31,51).rotated(-Math.PI/2),Math.toRadians(300)))
                            .build(),
                    new double[]{90,90,90,180}
            );
        } else {
            robot.sideClaw.barDown();
            robot.sideClaw.openClaw();
            robot.follow(0.1,0.7,
                    robot.path(Math.PI)
                            .splineTo(new Pose2d(GlobalConstants.BLUE_BLOCK_3.rotated(-Math.PI/2),Math.toRadians(245)))
                            .build(),
                    new double[]{180}, true, 6
            );
            robot.sideClaw.closeClaw();
            Thread.sleep(500);
            robot.sideClaw.barMid();

            robot.follow(0.2,0.7,
                    robot.path(Math.PI/2)
                            .lineTo(new Vector2d(-39,-56).rotated(-Math.PI/2)) // 72-18-9-3
                            .splineTo(new Pose2d(new Vector2d(-40,-24).rotated(-Math.PI/2),Math.toRadians(0))) // 72-18-9-3
                            .splineTo(new Pose2d(new Vector2d(-40,36).rotated(-Math.PI/2),Math.toRadians(0)))
                            .splineTo(new Pose2d(new Vector2d(-31,38).rotated(-Math.PI/2),Math.toRadians(300)))
                            .build(),
                    new double[]{90,90,90,180}
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
            robot.follow(0.25,0.9,
                    robot.path(Math.PI/2)
                            .splineTo(new Pose2d(new Vector2d(-40,36).rotated(-Math.PI/2),Math.toRadians(180)))
                            .splineTo(new Pose2d(new Vector2d(-40,-24).rotated(-Math.PI/2),Math.toRadians(180)))
                            .splineTo(new Pose2d(new Vector2d(-22,-48-8+4).rotated(-Math.PI/2),Math.toRadians(270)))
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
            robot.sideClaw.barUp();
            robot.sideClaw.openClaw();

            robot.intake.intakeIn();

            robot.follow(0.25,0.9,
                    robot.path(Math.PI/2)
                            .splineTo(new Pose2d(new Vector2d(-40,36).rotated(-Math.PI/2),Math.toRadians(180)))
                            .splineTo(new Pose2d(new Vector2d(-40,-24).rotated(-Math.PI/2),Math.toRadians(180)))
                            .splineTo(new Pose2d(new Vector2d(-22,-48+4).rotated(-Math.PI/2),Math.toRadians(270)))
                            .build(),
                    new double[]{180,180,180}
            );
            robot.follow(0.3,0.5,
                    robot.path(Math.PI)
                            .forward(8)
                            .build(),
                    new double[]{180},false
            );
        } else {
            robot.sideClaw.barDown();
            robot.sideClaw.openClaw();
            robot.follow(0.1,0.9,
                    robot.path(Math.PI/2)
                            .splineTo(new Pose2d(new Vector2d(-36,24).rotated(-Math.PI/2),Math.toRadians(180)))
                            .splineTo(new Pose2d(new Vector2d(-40,-12).rotated(-Math.PI/2),Math.toRadians(180)))
                            .splineTo(new Pose2d(GlobalConstants.BLUE_BLOCK_6.rotated(-Math.PI/2),Math.toRadians(200)))
                            .build(),
                    new double[]{180,180,180}, true, 6
            );

            robot.sideClaw.closeClaw();
            Thread.sleep(500);
            robot.sideClaw.barMid();
        }

        if(pos == 1) {
            robot.follow(0.2,0.7,
                    robot.path(Math.PI/2)
                            .lineTo(new Vector2d(-39,-32).rotated(-Math.PI/2)) // 72-18-9-3
                            .splineTo(new Pose2d(new Vector2d(-40,-24).rotated(-Math.PI/2),Math.toRadians(0))) // 72-18-9-3
                            .splineTo(new Pose2d(new Vector2d(-40,36).rotated(-Math.PI/2),Math.toRadians(0)))
                            .splineTo(new Pose2d(new Vector2d(-31,49).rotated(-Math.PI/2),Math.toRadians(270)))
                            .build(),
                    new double[]{90,90,90,180}
            );

            robot.sideClaw.barDown();
            Thread.sleep(100);
            robot.sideClaw.openClaw();
            Thread.sleep(1000);
            robot.rotateTo(90);
            robot.follow(0.1,0.5,
                    robot.path(Math.PI/2)
                            .back(6)
                            .build(),
                    new double[]{90}
            );

            robot.hook.hookDown();
            Thread.sleep(1000);
        } else {
            robot.follow(0.2,0.9,
                    robot.path(Math.PI/2)
                            .splineTo(new Pose2d(new Vector2d(-40,-24).rotated(-Math.PI/2),Math.toRadians(0))) // 72-18-9-3
                            .splineTo(new Pose2d(new Vector2d(-40,20).rotated(-Math.PI/2),Math.toRadians(0)))
                            .build(),
                    new double[]{180,270}
            );
            robot.intake.intakeOut();
            Thread.sleep(500);
            robot.rotateTo(90);
            robot.follow(0.1,0.8,
                    robot.path(0)
                            .splineTo(new Pose2d(new Vector2d(-40,31).rotated(-Math.PI/2),Math.toRadians(0)))
                            .splineTo(new Pose2d(new Vector2d(-31,49).rotated(-Math.PI/2),Math.toRadians(270)))
                            .splineTo(new Pose2d(new Vector2d(-28,49).rotated(-Math.PI/2),Math.toRadians(270)))
                            .build(),
                    new double[]{90,90,90}
            );
            robot.hook.hookDown();
            Thread.sleep(1000);


        }
        ElapsedTime gayshit = new ElapsedTime();
        while (gayshit.milliseconds() < 2000) {
            robot.drive.setDrivePower(1);
        }
        robot.rotateTo(270);
        robot.hook.hookUp();
        robot.follow(0.6,1,
                robot.path(Math.PI)
                        .strafeTo(new Vector2d(-63,0).rotated(-Math.PI/2))
                        .build(),
                new double[]{90}, false
        );

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
