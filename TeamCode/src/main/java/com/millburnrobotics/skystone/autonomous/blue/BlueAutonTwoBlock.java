package com.millburnrobotics.skystone.autonomous.blue;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.millburnrobotics.skystone.Constants;
import com.millburnrobotics.skystone.robot.MohanBot;
import com.millburnrobotics.skystone.util.BarkerClass;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.opencv.android.OpenCVLoader;


@Autonomous(group = "auton")
public class BlueAutonTwoBlock extends LinearOpMode {
    static {
        if(OpenCVLoader.initDebug()) {
            Log.d("opencv","yay it works");
        } else {
            Log.d("opencv","nope it doesnt work");
        }
    }
    private static double BOT_WIDTH = 18.0;
    private static double DEFAULT_STRAFE_POWER = 0.5;
    private static double DEFAULT_ROTATION_POWER = 0.5;
    private static double BASEPLATE_ALIGNMENT_Y = 49; //24 + 8 + 34/2

    @Override
    public void runOpMode() throws InterruptedException {
        //forwards, to wall, on first tile beyond bridge, sideplate on tile edge nearest to audience
        MohanBot robot = new MohanBot(hardwareMap, this, new Pose2d(-63, -2* Constants.FieldConstants.TILE_WIDTH + BOT_WIDTH/2, Math.toRadians(270)));

        BarkerClass barker = new BarkerClass(hardwareMap, Constants.SIDE.BLUE);


        robot.intake.intakeStop();
        robot.chainBar.closeClaw();

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

        if (pos == 1) {
            robot.moveTo(new Vector2d(-1.5* Constants.FieldConstants.TILE_WIDTH, -1.5* Constants.FieldConstants.TILE_WIDTH), 310,DEFAULT_STRAFE_POWER,DEFAULT_ROTATION_POWER);
            robot.intake.intakeIn();
            robot.strafeTo(new Vector2d(-20, -1* Constants.FieldConstants.TILE_WIDTH),DEFAULT_STRAFE_POWER);
        } else if (pos == 2) {
            robot.moveTo(new Vector2d(-1.5* Constants.FieldConstants.TILE_WIDTH, -1.5* Constants.FieldConstants.TILE_WIDTH-4), 310, DEFAULT_STRAFE_POWER,DEFAULT_ROTATION_POWER);
            robot.intake.intakeIn();
            robot.strafeTo(new Vector2d(-20, -1* Constants.FieldConstants.TILE_WIDTH-4),DEFAULT_STRAFE_POWER);
        } else {
            robot.moveTo(new Vector2d(-24, -24), 180,DEFAULT_STRAFE_POWER,DEFAULT_ROTATION_POWER);
            robot.intake.intakeIn();
            robot.strafeTo(new Vector2d(-24, -32),DEFAULT_STRAFE_POWER);
        }
        robot.strafeTo(new Vector2d(-40, -24), DEFAULT_STRAFE_POWER);
        robot.intake.intakeStop();
        robot.moveTo(new Vector2d(-40, 24), 270,DEFAULT_STRAFE_POWER,DEFAULT_ROTATION_POWER);

        robot.intake.intakeOut();
        Thread.sleep(1000);
        robot.intake.intakeStop();

        robot.moveTo(new Vector2d(-36, -36), 180,DEFAULT_STRAFE_POWER,DEFAULT_ROTATION_POWER);

        if (pos == 1) {
            robot.strafeTo(new Vector2d(-24,-36),0.5);
            robot.intake.intakeIn();
            robot.strafeTo(new Vector2d(-24,-48),0.3);
        } else if (pos == 2) {
            robot.strafeTo(new Vector2d(-36,-36-7),0.5);
            robot.strafeTo(new Vector2d(-24,-36-7),0.5);
            robot.intake.intakeIn();
            robot.strafeTo(new Vector2d(-24,-48-7),0.3);
        } else {
            robot.strafeTo(new Vector2d(-36,-36-14),0.5);
            robot.strafeTo(new Vector2d(-24,-36-14),0.5);
            robot.intake.intakeIn();
            robot.strafeTo(new Vector2d(-24,-48-14),0.3);
        }
        robot.strafeTo(new Vector2d(-40,-36), DEFAULT_STRAFE_POWER);
        robot.moveTo(new Vector2d(-40,20), 270, DEFAULT_STRAFE_POWER, DEFAULT_ROTATION_POWER);

        robot.intake.intakeOut();
        Thread.sleep(1000);
        robot.intake.intakeStop();
        robot.strafeTo(new Vector2d(-36,0),0.5);
    }
}
