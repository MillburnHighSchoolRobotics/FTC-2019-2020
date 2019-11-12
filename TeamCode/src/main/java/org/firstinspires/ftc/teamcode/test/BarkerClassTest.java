package org.firstinspires.ftc.teamcode.test;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robot.MohanBot;
import org.firstinspires.ftc.teamcode.threads.PositionMonitor;
import org.firstinspires.ftc.teamcode.threads.ThreadManager;
import org.firstinspires.ftc.teamcode.util.BarkerClass;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;


@Autonomous(group = "auton")
public class BarkerClassTest extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        MohanBot mohanBot = new MohanBot(hardwareMap,this);

        mohanBot.getHook().hookUp();
        mohanBot.getChainBar().openClaw();

        MohanBot drive = new MohanBot(hardwareMap,this);
        BarkerClass barker = new BarkerClass(hardwareMap);

        barker.setupCam();

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectory(
                drive.trajectoryBuilder()
                        .strafeTo(new Vector2d(7.5, 3))
                        .build()
        );

        Thread.sleep(5000);

        Log.d("BarkerClass Block Pos:",barker.getPos() + "");
    }
}
