package org.firstinspires.ftc.teamcode.autonomous.blue;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.heading.SplineInterpolator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.MohanBot;

import static org.firstinspires.ftc.teamcode.robot.GlobalConstants.*;


@Autonomous(group = "auton")
public class BlueAutonBaseplate extends LinearOpMode {

    private static double BOT_WIDTH = 18.0;
    private static double DEFAULT_STRAFE_POWER = 0.75;
    private static double DEFAULT_ROTATION_POWER = 0.25;
    private static double BASEPLATE_ALIGNMENT_Y = 49; //24 + 8 + 34/2

    @Override
    public void runOpMode() throws InterruptedException {
        //backwards, to wall, sideplate on tile edge nearest to bridge
        MohanBot robot = new MohanBot(hardwareMap,this,new Pose2d(-63, TILE_WIDTH + BOT_WIDTH/2, Math.toRadians(90)));

        waitForStart();

        if (isStopRequested()) return;

        robot.moveTo(new Vector2d(-48, BASEPLATE_ALIGNMENT_Y), 90, DEFAULT_STRAFE_POWER, DEFAULT_ROTATION_POWER);

        Thread.sleep(1000);

        robot.moveTo(new Vector2d(-24, BASEPLATE_ALIGNMENT_Y), 90, DEFAULT_STRAFE_POWER, DEFAULT_ROTATION_POWER);

        //robot.hook.hookDown();
        Thread.sleep(1000);

        robot.moveTo(new Vector2d(-63, BASEPLATE_ALIGNMENT_Y), 90, DEFAULT_STRAFE_POWER, DEFAULT_ROTATION_POWER);

        //robot.hook.hookUp();
        Thread.sleep(1000);
//
        robot.moveTo(new Vector2d(-63, 0), 90, DEFAULT_STRAFE_POWER, DEFAULT_ROTATION_POWER);
    }
}
