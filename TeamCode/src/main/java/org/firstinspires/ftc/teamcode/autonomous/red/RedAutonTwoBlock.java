package org.firstinspires.ftc.teamcode.autonomous.red;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.robot.MohanBot;
import org.firstinspires.ftc.teamcode.util.BarkerClass;
import org.opencv.android.OpenCVLoader;

import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
import static org.firstinspires.ftc.teamcode.robot.GlobalConstants.*;


@Autonomous(group = "auton")
public class RedAutonTwoBlock extends LinearOpMode {
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
        BarkerClass barker = new BarkerClass(hardwareMap, SIDE.RED);

        MohanBot robot = new MohanBot(hardwareMap, this, new Pose2d(63, -2*TILE_WIDTH + BOT_WIDTH/2, Math.toRadians(90)));

        robot.intake.intakeStop();

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
            robot.moveTo(new Vector2d(1.5*TILE_WIDTH, -1.5*TILE_WIDTH), 50,DEFAULT_STRAFE_POWER,DEFAULT_ROTATION_POWER);
//            robot.strafeTo(new Vector2d(1.5*TILE_WIDTH, -1.5*TILE_WIDTH),DEFAULT_STRAFE_POWER);
//            telemetry.addData("fuck","fuck im done moving");
//            telemetry.update();
//
//            robot.rotateTo(50);
//            telemetry.addData("fuck","fuck im done turning");
//            telemetry.update();
            robot.intake.intakeIn();
            robot.strafeTo(new Vector2d(20, -1*TILE_WIDTH),DEFAULT_STRAFE_POWER);
        }
        else if (pos == 2) {
            robot.moveTo(new Vector2d(1.5*TILE_WIDTH, -1.5*TILE_WIDTH-4), 50, DEFAULT_STRAFE_POWER,DEFAULT_ROTATION_POWER);
            robot.intake.intakeIn();
            robot.strafeTo(new Vector2d(20, -1*TILE_WIDTH-4),DEFAULT_STRAFE_POWER);
        } else {
            robot.moveTo(new Vector2d(24, -24), 180,DEFAULT_STRAFE_POWER,DEFAULT_ROTATION_POWER);
            robot.intake.intakeIn();
            robot.strafeTo(new Vector2d(24, -32),DEFAULT_STRAFE_POWER);
        }
        robot.intake.intakeStop();
        robot.moveTo(new Vector2d(48, -24), 0,DEFAULT_STRAFE_POWER,DEFAULT_ROTATION_POWER);
        robot.moveTo(new Vector2d(48, 24), 0,DEFAULT_STRAFE_POWER,DEFAULT_ROTATION_POWER);

        robot.intake.intakeOut();
        Thread.sleep(1000);
        robot.intake.intakeStop();


        //second block
//        if (pos == 1) {
//            robot.moveTo(new Vector2d(36, -1.5*TILE_WIDTH+4), 180,DEFAULT_STRAFE_POWER,0.4);
//            robot.moveTo(new Vector2d(TILE_WIDTH, -1.5*TILE_WIDTH+4), 180,DEFAULT_STRAFE_POWER,0.4);
//            robot.intake.intakeIn();
//            robot.strafeTo(new Vector2d(TILE_WIDTH, -1.5*TILE_WIDTH-4),DEFAULT_STRAFE_POWER);
//        }
//        else if (pos == 2) { //OFFSET BECAUSE OF DRIFT IN MOVEMENT
//            robot.moveTo(new Vector2d(40, -1.5*TILE_WIDTH-4), 180,DEFAULT_STRAFE_POWER,0.4);
//            robot.moveTo(new Vector2d(TILE_WIDTH+4, -1.5*TILE_WIDTH-4), 180,DEFAULT_STRAFE_POWER,0.4);
//            robot.intake.intakeIn();
//            robot.strafeTo(new Vector2d(TILE_WIDTH+4, -1.5*TILE_WIDTH-12),DEFAULT_STRAFE_POWER);
//        }
//        else {
//            robot.moveTo(new Vector2d(40, -1.5*TILE_WIDTH-12), 180,DEFAULT_STRAFE_POWER,0.4);
//            robot.moveTo(new Vector2d(TILE_WIDTH+4, -1.5*TILE_WIDTH-12), 180,DEFAULT_STRAFE_POWER,0.4);
//            robot.intake.intakeIn();
//            robot.strafeTo(new Vector2d(TILE_WIDTH+4, -1.5*TILE_WIDTH-20),DEFAULT_STRAFE_POWER);
//        }
//
//        robot.intake.intakeStop();
//        robot.moveTo(new Vector2d(48, -24), 0,0.6,0.4);
//        robot.moveTo(new Vector2d(48, 24), 0,0.6,0.4);
//
//        robot.intake.intakeOut();
//        Thread.sleep(1000);
    }
}
