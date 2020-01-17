package org.firstinspires.ftc.teamcode.test;

import android.app.Activity;
import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.internal.opmode.OpModeManagerImpl;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.threads.CollisionMonitor;
import org.firstinspires.ftc.teamcode.threads.PositionMonitor;
import org.firstinspires.ftc.teamcode.threads.ThreadManager;

@Autonomous(group = "util")
public class CollisionMonitorTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        ThreadManager manager = ThreadManager.getInstance();
        manager.setHardwareMap(hardwareMap);
        manager.setupThread("CollisionMonitor", CollisionMonitor.class);
        waitForStart();

        if (isStopRequested()) return;

        while (!shouldStop()) {
            telemetry.addData("collision", ThreadManager.getInstance().getValue("collision", Boolean.class));
            telemetry.addData("currentJerkX", ThreadManager.getInstance().getValue("currentJerkX", Double.class));
            telemetry.addData("currentJerkY", ThreadManager.getInstance().getValue("currentJerkY", Double.class));
            telemetry.update();

            Log.d("collision", ThreadManager.getInstance().getValue("collision", Boolean.class)+"");
            Log.d("currentJerkX", ThreadManager.getInstance().getValue("currentJerkX", Double.class)+"");
            Log.d("currentJerkY", ThreadManager.getInstance().getValue("currentJerkY", Double.class)+"");
            Thread.sleep(500);
        }
    }
    public static boolean shouldStop() {
        Activity currActivity = AppUtil.getInstance().getActivity();
        OpModeManagerImpl manager = OpModeManagerImpl.getOpModeManagerOfActivity(currActivity);
        OpMode currentOpMode = manager.getActiveOpMode();
        return currentOpMode instanceof LinearOpMode &&
                ((LinearOpMode) currentOpMode).isStarted() &&
                ((LinearOpMode) currentOpMode).isStopRequested();
    }
}