package org.firstinspires.ftc.teamcode.autonomous.red;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.heading.ConstantInterpolator;
import com.acmerobotics.roadrunner.path.heading.LinearInterpolator;
import com.acmerobotics.roadrunner.path.heading.SplineInterpolator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.robot.GlobalConstants;
import org.firstinspires.ftc.teamcode.robot.MohanBot;
import org.firstinspires.ftc.teamcode.util.FUCKMYLIFEClass;
import org.firstinspires.ftc.teamcode.util.VuforiaLocalizerImplSubclass;
import org.opencv.android.OpenCVLoader;

import static org.firstinspires.ftc.teamcode.robot.GlobalConstants.CHAINBAR_OUT_TICKS;

@Autonomous(group = "auton")
public class RedAutonDoubleMineral extends LinearOpMode {
    static {
        if(OpenCVLoader.initDebug()) {
            Log.d("opencv","yay it works");
        } else {
            Log.d("opencv","nope it doesnt work");
        }
    }
    @Override
    public void runOpMode() throws InterruptedException {
        MohanBot mohanBot = new MohanBot(hardwareMap,this);

        mohanBot.setPose(new Pose2d(-39, -63, Math.PI/2));

        mohanBot.getHook().hookUp();

        mohanBot.getChainBar().openClaw();

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

        FUCKMYLIFEClass fuck = new FUCKMYLIFEClass(vuforiaInstance);

        int skystone = fuck.getPos();
        telemetry.addData("cv",skystone+"");
        telemetry.update();

        mohanBot.getChainBar().chainBarUp();
        mohanBot.getIntake().intakeIn();
        mohanBot.getChainBar().normalClaw();
        switch(skystone) {
            case 1:
                mohanBot.followTrajectory(
                        mohanBot.trajectoryBuilder()
                                .splineTo(new Pose2d(-30,-24,Math.toRadians(45)),new LinearInterpolator(Math.PI/2,-Math.PI/4))
                                .build()
                );

                mohanBot.followTrajectory(
                        mohanBot.trajectoryBuilder()
                                .splineTo(new Pose2d(-20,-44,Math.PI/2),new LinearInterpolator(Math.toRadians(45),-3*Math.PI/4))
                                .build()
                );
                mohanBot.getChainBar().chainBarIn();
                mohanBot.turnTo(3*Math.PI/2);
                mohanBot.followTrajectory(
                        mohanBot.trajectoryBuilder()
                                .strafeTo(new Vector2d(60,-44))
                                .build()
                );
                mohanBot.followTrajectory(
                        mohanBot.trajectoryBuilder()
                                .strafeTo(new Vector2d(60,-44))
                                .build()
                );
                break;
            case 3:
                mohanBot.followTrajectory(
                        mohanBot.trajectoryBuilder()
                                .splineTo(new Pose2d(-38,-24,Math.toRadians(45)),new LinearInterpolator(Math.PI/2,-Math.PI/4))
                                .build()
                );
                Thread.sleep(100);
                mohanBot.getIntake().intakeStop();
                mohanBot.getChainBar().chainBarIn();

                mohanBot.followTrajectory(
                        mohanBot.trajectoryBuilder()
                                .back(6)
                                .splineTo(new Pose2d(-24,-44,3*Math.PI/2),new LinearInterpolator(Math.toRadians(45),-3*Math.PI/4))
                                .build()
                );
                mohanBot.turnTo(3*Math.PI/2);
                mohanBot.followTrajectory(
                        mohanBot.trajectoryBuilder()
                                .strafeTo(new Vector2d(60,-44))
                                .build()
                );
                mohanBot.followTrajectory(
                        mohanBot.trajectoryBuilder()
                                .strafeTo(new Vector2d(60,-44))
                                .build()
                );
                break;
            case 2:
                mohanBot.followTrajectory(
                        mohanBot.trajectoryBuilder()
                                .forward(10)
                                .splineTo(new Pose2d(-40,-24,Math.toRadians(135)),new SplineInterpolator(Math.PI/2,Math.toRadians(135)))
                                .build()
                );
                Thread.sleep(100);
                mohanBot.getChainBar().chainBarIn();
                mohanBot.followTrajectory(
                        mohanBot.trajectoryBuilder()
                                .reverse()
                                .splineTo(new Pose2d(0,-40,Math.PI))
                                .splineTo(new Pose2d(52,-30,3*Math.PI/2), new SplineInterpolator(3*Math.PI/2,Math.PI))
                                .strafeTo(new Vector2d(52,-22))
                                .build()
                );
                break;
        }
        mohanBot.getChainBar().closeClaw();
        mohanBot.turnTo(3*Math.PI/2);
        mohanBot.followTrajectory(
                mohanBot.trajectoryBuilder()
                        .setReversed(false)
                        .back(18)
                        .build()
        );
        Thread.sleep(100);
        mohanBot.getHook().hookDown();
        mohanBot.getChainBar().chainBarOut();
        mohanBot.getIntake().intakeStop();
        mohanBot.followTrajectory(
                mohanBot.trajectoryBuilder()
                        .forward(64)
                        .build()
        );
        mohanBot.getChainBar().openClaw();
        Thread.sleep(500);
        mohanBot.getChainBar().chainBarIn();
        mohanBot.getHook().hookUp();
        mohanBot.followTrajectory(
                mohanBot.trajectoryBuilder()
                        .strafeTo(new Vector2d(12,-63))
                        .build()
        );
        mohanBot.followTrajectory(
                mohanBot.trajectoryBuilder()
                        .reverse()
                        .splineTo(new Pose2d(0,-40,0), new SplineInterpolator(Math.PI,3*Math.PI/2))
                        .build()
        );


        switch(skystone) {
            case 1:
                mohanBot.followTrajectory(
                        mohanBot.trajectoryBuilder()
                                .splineTo(new Pose2d(-40,-22,Math.PI/2), new ConstantInterpolator(Math.PI))
                                .build()
                );
                mohanBot.followTrajectory(
                        mohanBot.trajectoryBuilder()
                                .strafeTo(new Vector2d(-40,-22))
                                .build()
                );
                mohanBot.getChainBar().chainBarUp();
                mohanBot.getIntake().intakeIn();
                mohanBot.getChainBar().openClaw();
                mohanBot.followTrajectory(
                        mohanBot.trajectoryBuilder()
                                .forward(8)
                                .build()
                );
                break;
            case 3:
                mohanBot.followTrajectory(
                        mohanBot.trajectoryBuilder()
                                .splineTo(new Pose2d(-48,-22,Math.PI/2), new ConstantInterpolator(Math.PI))
                                .build()
                );
                mohanBot.followTrajectory(
                        mohanBot.trajectoryBuilder()
                                .strafeTo(new Vector2d(-48,-22))
                                .build()
                );
                mohanBot.getChainBar().chainBarUp();
                mohanBot.getIntake().intakeIn();
                mohanBot.getChainBar().openClaw();
                mohanBot.followTrajectory(
                        mohanBot.trajectoryBuilder()
                                .forward(8)
                                .build()
                );
                mohanBot.getChainBar().openClaw();
                break;
            case 2:
                mohanBot.followTrajectory(
                        mohanBot.trajectoryBuilder()
                                .splineTo(new Pose2d(-56,-22,Math.PI/2), new ConstantInterpolator(Math.PI))
                                .build()
                );
                mohanBot.followTrajectory(
                        mohanBot.trajectoryBuilder()
                                .strafeTo(new Vector2d(-56,-22))
                                .build()
                );
                mohanBot.getChainBar().chainBarUp();
                mohanBot.getIntake().intakeIn();
                mohanBot.getChainBar().openClaw();
                mohanBot.followTrajectory(
                        mohanBot.trajectoryBuilder()
                                .forward(8)
                                .build()
                );
                break;
        }
        mohanBot.getChainBar().chainBarIn();
        mohanBot.followTrajectory(
                mohanBot.trajectoryBuilder()
                        .strafeLeft(18)
                        .build()
        );
        mohanBot.getChainBar().closeClaw();
        mohanBot.followTrajectory(
                mohanBot.trajectoryBuilder()
                        .strafeTo(new Vector2d(40,-40))
                        .build()
        );
        mohanBot.getChainBar().chainBarOut();
        mohanBot.turnTo(Math.PI);
        while (mohanBot.getChainBar().isBusy()) {
            Thread.sleep(10);
        }
        mohanBot.getChainBar().openClaw();
        Thread.sleep(500);
        mohanBot.getChainBar().chainBarIn();
        mohanBot.followTrajectory(
                mohanBot.trajectoryBuilder()
                        .strafeTo(new Vector2d(0,-40))
                        .build()
        );
    }
}
