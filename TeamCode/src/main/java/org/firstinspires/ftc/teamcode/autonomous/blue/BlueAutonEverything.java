package org.firstinspires.ftc.teamcode.autonomous.blue;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.heading.ConstantInterpolator;
import com.acmerobotics.roadrunner.path.heading.LinearInterpolator;
import com.acmerobotics.roadrunner.path.heading.SplineInterpolator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.robot.MohanBot;
import org.firstinspires.ftc.teamcode.util.BarkerClass;

import static org.firstinspires.ftc.teamcode.robot.GlobalConstants.CHAINBAR_OUT_TICKS;

@Autonomous(group = "auton")
public class BlueAutonEverything extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        MohanBot mohanBot = new MohanBot(hardwareMap,this);

        mohanBot.setPose(new Pose2d(-39, 63, 3*Math.PI/2));

        mohanBot.getHook().hookUp();
        mohanBot.getChainBar().openClaw();
        mohanBot.getChainBar().normalClaw();

        waitForStart();
        if (isStopRequested()) return;

        int skystone = 3;//barker.getPos();;//(int)Math.round(3*Math.random())+1;
        telemetry.addData("barkerclass",skystone+"");
        telemetry.update();

        mohanBot.getChainBar().chainBarUp();
        mohanBot.getIntake().intakeIn();
        switch(skystone) {
            case 1:
                mohanBot.followTrajectory(
                        mohanBot.trajectoryBuilder()
                                .splineTo(new Pose2d(-30,24,Math.toRadians(315)),new LinearInterpolator(3*Math.PI/2,Math.PI/4))
                                .build()
                );
                mohanBot.getChainBar().chainBarIn();
                mohanBot.followTrajectory(
                        mohanBot.trajectoryBuilder()
                                .splineTo(new Pose2d(-12,40,0),new LinearInterpolator(Math.toRadians(315),Math.PI/4))
                                .splineTo(new Pose2d(52,30,3*Math.PI/2), new LinearInterpolator(0,Math.PI/2))
                                .strafeTo(new Vector2d(52,22))
                                .build()
                );
                break;
            case 2:
                mohanBot.followTrajectory(
                        mohanBot.trajectoryBuilder()
                                .splineTo(new Pose2d(-38,24,Math.toRadians(315)),new LinearInterpolator(3*Math.PI/2,Math.PI/4))
                                .build()
                );
                mohanBot.getChainBar().chainBarIn();
                mohanBot.followTrajectory(
                        mohanBot.trajectoryBuilder()
                                .splineTo(new Pose2d(-12,40,0),new LinearInterpolator(Math.toRadians(315),Math.PI/4))
                                .splineTo(new Pose2d(52,30,3*Math.PI/2), new LinearInterpolator(0,Math.PI/2))
                                .strafeTo(new Vector2d(52,22))
                                .build()
                );
                break;
            case 3:
                mohanBot.followTrajectory(
                        mohanBot.trajectoryBuilder()
                                .splineTo(new Pose2d(-40,26,Math.toRadians(225)),new SplineInterpolator(3*Math.PI/2,Math.toRadians(225)))
                                .build()
                );
                mohanBot.getChainBar().chainBarIn();
                mohanBot.followTrajectory(
                        mohanBot.trajectoryBuilder()
                                .reverse()
                                .splineTo(new Pose2d(0,40,Math.PI))
                                .splineTo(new Pose2d(52,30,Math.PI/2), new SplineInterpolator(Math.PI/2,Math.PI))
                                .strafeTo(new Vector2d(52,22))
                                .build()
                );
                break;
        }
        mohanBot.turnTo(Math.PI/2);
        mohanBot.followTrajectory(
                mohanBot.trajectoryBuilder()
                        .setReversed(false)
                        .back(18)
                        .build()
        );

        mohanBot.getHook().hookDown();
        mohanBot.getChainBar().closeClaw();
        Thread.sleep(100);
        mohanBot.getChainBar().chainBarOut();
        mohanBot.getIntake().intakeStop();
        mohanBot.followTrajectory(
                mohanBot.trajectoryBuilder()
                        .forward(48)
                        .build()
        );
        mohanBot.getChainBar().openClaw();
        mohanBot.getChainBar().chainBarIn();
        mohanBot.getHook().hookUp();
        mohanBot.followTrajectory(
                mohanBot.trajectoryBuilder()
                        .strafeTo(new Vector2d(12,63))
                        .build()
        );
        mohanBot.followTrajectory(
                mohanBot.trajectoryBuilder()
                        .reverse()
                        .splineTo(new Pose2d(0,40,0), new SplineInterpolator(Math.PI,Math.PI/2))
                        .build()
        );
//        switch(skystone) {
//            case 1:
//                mohanBot.followTrajectory(
//                        mohanBot.trajectoryBuilder()
//                                .splineTo(new Pose2d(-36,22,3*Math.PI/2), new ConstantInterpolator(Math.PI))
//                                .lineTo(new Vector2d(-42,22), new ConstantInterpolator(Math.PI))
//                                .build()
//                );
//                break;
//            case 2:
//                mohanBot.followTrajectory(
//                        mohanBot.trajectoryBuilder()
//                                .splineTo(new Pose2d(-44,22,3*Math.PI/2), new ConstantInterpolator(Math.PI))
//                                .lineTo(new Vector2d(-50,22), new ConstantInterpolator(Math.PI))
//                                .build()
//                );
//                break;
//            case 3:
//                mohanBot.followTrajectory(
//                        mohanBot.trajectoryBuilder()
//                                .splineTo(new Pose2d(-52,22,3*Math.PI/2), new ConstantInterpolator(Math.PI))
//                                .build()
//                );
//                mohanBot.getIntake().intakeIn();
//                mohanBot.getChainBar().chainBarUp();
//                mohanBot.followTrajectory(
//                        mohanBot.trajectoryBuilder()
//                                .lineTo(new Vector2d(-58,22), new ConstantInterpolator(Math.PI))
//                                .build()
//                );
//                break;
//        }
//        mohanBot.getChainBar().chainBarIn();
//        mohanBot.getIntake().intakeStop();
//        mohanBot.followTrajectory(
//                mohanBot.trajectoryBuilder()
//                        .strafeRight(18)
//                        .lineTo(new Vector2d(24,40), new ConstantInterpolator(Math.PI))
//                        .build()
//        );
//        mohanBot.getChainBar().closeClaw();
//        Thread.sleep(50);
//        mohanBot.getChainBar().chainBarTo(CHAINBAR_OUT_TICKS);
//        mohanBot.getChainBar().openClaw();
//        Thread.sleep(50);
//        mohanBot.getChainBar().closeClaw();
//        mohanBot.getChainBar().chainBarIn();
//        mohanBot.followTrajectory(
//                mohanBot.trajectoryBuilder()
//                        .lineTo(new Vector2d(0,40), new ConstantInterpolator(Math.PI))
//                        .build()
//        );
    }
}
