package org.firstinspires.ftc.teamcode.autonomous.blue;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.MohanBot;
import org.opencv.android.OpenCVLoader;

import static org.firstinspires.ftc.teamcode.robot.GlobalConstants.BLUE_BLOCK_1;
import static org.firstinspires.ftc.teamcode.robot.GlobalConstants.BLUE_BLOCK_2;
import static org.firstinspires.ftc.teamcode.robot.GlobalConstants.BLUE_BLOCK_3;
import static org.firstinspires.ftc.teamcode.robot.GlobalConstants.BLUE_BLOCK_4;
import static org.firstinspires.ftc.teamcode.robot.GlobalConstants.BLUE_BRIDGE_PARK;
import static org.firstinspires.ftc.teamcode.robot.GlobalConstants.BLUE_DELIVERY_1;
import static org.firstinspires.ftc.teamcode.robot.GlobalConstants.BLUE_DELIVERY_2;
import static org.firstinspires.ftc.teamcode.robot.GlobalConstants.BLUE_DELIVERY_3;
import static org.firstinspires.ftc.teamcode.robot.GlobalConstants.BLUE_DELIVERY_4;
import static org.firstinspires.ftc.teamcode.robot.GlobalConstants.BLUE_FOUNDATION;
import static org.firstinspires.ftc.teamcode.robot.GlobalConstants.BOT_LENGTH;
import static org.firstinspires.ftc.teamcode.robot.GlobalConstants.BOT_WIDTH;


@Autonomous(group = "auton")
public class BlueAutonNew extends LinearOpMode {
    static {
        if (OpenCVLoader.initDebug()) {
            Log.d("opencv", "yay it works");
        } else {
            Log.d("opencv", "nope it doesnt work");
        }
    }

    MohanBot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new MohanBot(hardwareMap, this, (new Pose2d(new Vector2d(-72+BOT_WIDTH/2.0,-48+BOT_LENGTH/2.0), 0)));

        waitForStart();

        if (isStopRequested()) return;

        int pos = 1;
        telemetry.addData("block",pos);
        telemetry.update();

        if (pos == 1) {
            sample12(BLUE_BLOCK_4);
            deliver(BLUE_DELIVERY_1);

            cycle(BLUE_BLOCK_1);
            deliver(BLUE_DELIVERY_2);

            cycle(BLUE_BLOCK_2);
            deliver(BLUE_DELIVERY_3);

            cycle(BLUE_BLOCK_3);
            deliver(BLUE_DELIVERY_4);
        } else if (pos == 2) {

        }
        park();
    }
    public void sample12(Vector2d block) {
        robot.follow(0.1,1,
                robot.path(Math.toRadians(270))
                        .lineTo(new Vector2d(-72+BOT_WIDTH/2.0+4,-48+BOT_LENGTH/2.0).rotated(-Math.PI/2))
                        .splineTo(new Pose2d(block.rotated(-Math.PI/2),Math.toRadians(270)))
                        .build(),
                new double[]{0,0}, true, 10
        );
    }
    public void cycle(Vector2d block) {
        robot.follow(0.1,1,
                robot.path(Math.toRadians(170))
                        .splineTo(new Pose2d(new Vector2d(-38,0).rotated(-Math.PI/2),Math.toRadians(180)))
                        .splineTo(new Pose2d(block.rotated(-Math.PI/2),Math.toRadians(190)))
                        .build(),
                new double[]{0,0}, true, 10
        );
    }
    public void deliver(Vector2d deliver) {
        robot.follow(0.2,1,
                robot.path(Math.toRadians(10))
                        .splineTo(new Pose2d(new Vector2d(-38,0).rotated(-Math.PI/2),Math.toRadians(180)))
                        .splineTo(new Pose2d(deliver.rotated(-Math.PI/2),Math.toRadians(345)))
                        .build(),
                new double[]{0,0}
        );
    }
    public void park() {
        robot.follow(0.2,1,
                robot.path(Math.toRadians(0))
                        .lineTo(BLUE_FOUNDATION.rotated(-Math.PI/2))
                        .build(),
                new double[]{90}, true, 10
        );
        robot.follow(0.3,1,
                robot.path(Math.toRadians(90))
                        .back(10)
                        .build(),
                new double[]{90}
        );
        robot.follow(0.2,1,
                robot.path(Math.toRadians(0))
                        .lineTo(new Vector2d(-72+BOT_WIDTH/2.0+4,BLUE_FOUNDATION.getY()).rotated(-Math.PI/2))
                        .build(),
                new double[]{180}
        );
        robot.follow(0.5,1,
                robot.path(Math.toRadians(180))
                        .splineTo(new Pose2d(BLUE_BRIDGE_PARK.rotated(-Math.PI/2),Math.toRadians(180)))
                        .build(),
                new double[]{90}, false
        );
    }
}
