package org.firstinspires.ftc.teamcode.autonomous.blue;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.heading.LinearInterpolator;
import com.acmerobotics.roadrunner.path.heading.SplineInterpolator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.robot.GlobalConstants;
import org.firstinspires.ftc.teamcode.robot.MohanBot;
import org.firstinspires.ftc.teamcode.util.SkystoneDetector;
import org.firstinspires.ftc.teamcode.util.VuforiaLocalizerImplSubclass;
import org.opencv.android.OpenCVLoader;

@Autonomous(group = "auton")
public class BlueAutonSample extends LinearOpMode {
    static {
        if(OpenCVLoader.initDebug()) {
            Log.d("opencv","yay it works");
        } else {
            Log.d("opencv","nope it doesnt work");
        }
    }
    @Override
    public void runOpMode() throws InterruptedException {
        MohanBot mohanBot = new MohanBot(hardwareMap,this,new Pose2d(-39, 63, 3*Math.PI/2));

        mohanBot.hook.hookUp();
        mohanBot.chainBar.openClaw();

        VuforiaLocalizer.Parameters params = new VuforiaLocalizer.Parameters(hardwareMap.appContext.getResources().getIdentifier("Webcam1", "id", hardwareMap.appContext.getPackageName()));

        params.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        params.vuforiaLicenseKey = GlobalConstants.VUFORIA_KEY;
        telemetry.addData("Vuforia Status: ", "Loading...");
        telemetry.update();
        VuforiaLocalizerImplSubclass vuforiaInstance = new VuforiaLocalizerImplSubclass(params);

        telemetry.addData("Vuforia Status: ", "Loaded!");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        SkystoneDetector fuck = new SkystoneDetector(vuforiaInstance);

        int skystone = fuck.getPos();
        telemetry.addData("cv",skystone+"");
        telemetry.update();

        mohanBot.chainBar.chainBarUp();
        mohanBot.intake.intakeIn();
        mohanBot.chainBar.normalClaw();
        switch(skystone) {
            case 1:
                mohanBot.follow(
                        mohanBot.path()
                                .splineTo(new Pose2d(-30,24,Math.toRadians(315)),new LinearInterpolator(3*Math.PI/2,Math.PI/4))
                                .build()
                );

                mohanBot.follow(
                        mohanBot.path()
                                .splineTo(new Pose2d(-20,44,Math.PI/2),new LinearInterpolator(Math.toRadians(315),3*Math.PI/4))
                                .build()
                );
                Thread.sleep(200);
                mohanBot.chainBar.chainBarIn();
                mohanBot.rotateTo(Math.PI/2);
                Thread.sleep(100);
                mohanBot.follow(
                        mohanBot.path()
                                .strafeTo(new Vector2d(60,44))
                                .build()
                );
                mohanBot.follow(
                        mohanBot.path()
                                .strafeTo(new Vector2d(60,44))
                                .build()
                );
                break;
            case 2:
                mohanBot.follow(
                        mohanBot.path()
                                .splineTo(new Pose2d(-38,24,Math.toRadians(315)),new LinearInterpolator(3*Math.PI/2,Math.PI/4))
                                .build()
                );
                Thread.sleep(100);
                mohanBot.intake.intakeStop();
                mohanBot.follow(
                        mohanBot.path()
                                .back(6)
                                .splineTo(new Pose2d(-24,44,Math.PI/2),new LinearInterpolator(Math.toRadians(315),3*Math.PI/4))
                                .build()
                );
                Thread.sleep(200);
                mohanBot.chainBar.chainBarIn();
                mohanBot.rotateTo(Math.PI/2);
                Thread.sleep(100);
                mohanBot.follow(
                        mohanBot.path()
                                .strafeTo(new Vector2d(60,44))
                                .build()
                );
                mohanBot.follow(
                        mohanBot.path()
                                .strafeTo(new Vector2d(60,44))
                                .build()
                );
                break;
            case 3:
                mohanBot.follow(
                        mohanBot.path()
                                .forward(10)
                                .splineTo(new Pose2d(-40,24,Math.toRadians(225)),new SplineInterpolator(3*Math.PI/2,Math.toRadians(225)))
                                .build()
                );
                Thread.sleep(100);
                mohanBot.chainBar.chainBarIn();
                Thread.sleep(200);
                mohanBot.follow(
                        mohanBot.path()
                                .reverse()
                                .splineTo(new Pose2d(0,40,Math.PI))
                                .splineTo(new Pose2d(52,30,Math.PI/2), new SplineInterpolator(Math.PI/2,Math.PI))
                                .strafeTo(new Vector2d(52,22))
                                .build()
                );
                break;
        }
        mohanBot.chainBar.closeClaw();
        mohanBot.follow(
                mohanBot.path()
                        .setReversed(false)
                        .back(18)
                        .build()
        );
        Thread.sleep(100);
        mohanBot.hook.hookDown();
        mohanBot.chainBar.chainBarOut();
        mohanBot.intake.intakeStop();
        mohanBot.follow(
                mohanBot.path()
                        .forward(64)
                        .build()
        );
        mohanBot.chainBar.openClaw();
        Thread.sleep(700);
        mohanBot.chainBar.chainBarTo(GlobalConstants.CHAINBAR_IN_TICKS);;
        mohanBot.hook.hookUp();
        mohanBot.follow(
                mohanBot.path()
                        .strafeTo(new Vector2d(12,63))
                        .build()
        );
        mohanBot.follow(
                mohanBot.path()
                        .reverse()
                        .splineTo(new Pose2d(0,40,0), new SplineInterpolator(Math.PI,Math.PI/2))
                        .build()
        );
    }
}
