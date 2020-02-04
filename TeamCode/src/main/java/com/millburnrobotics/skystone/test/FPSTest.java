package com.millburnrobotics.skystone.test;

import android.app.Activity;

import com.millburnrobotics.lib.geometry.Pose;
import com.millburnrobotics.skystone.Constants;
import com.millburnrobotics.skystone.threads.PositionMonitor;
import com.millburnrobotics.skystone.threads.ThreadManager;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.internal.opmode.OpModeManagerImpl;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

@Autonomous(group = "util")
public class FPSTest extends LinearOpMode {

    DcMotorEx er;
    DcMotorEx el;
    DcMotorEx eb;

    @Override
    public void runOpMode() throws InterruptedException {
        er = (DcMotorEx) hardwareMap.dcMotor.get(Constants.DriveConstants._LeftFrontMotor);
        el = (DcMotorEx) hardwareMap.dcMotor.get(Constants.DriveConstants._RightBackMotor);
        eb = (DcMotorEx) hardwareMap.dcMotor.get(Constants.DriveConstants._RightFrontMotor);
        er.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        er.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        el.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        el.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        eb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        eb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        ThreadManager manager = ThreadManager.getInstance();
        manager.setHardwareMap(hardwareMap);
        manager.setupThread("PositionMonitor", PositionMonitor.class, new Pose(-63,-39,3*Math.PI/2));
        waitForStart();

        if (isStopRequested()) return;

        while (!shouldStop()) {
            telemetry.addData("heading", Math.toDegrees(ThreadManager.getInstance().getValue("yaw", Double.class)));
            telemetry.addData("x", ThreadManager.getInstance().getValue("x", Double.class));
            telemetry.addData("y", ThreadManager.getInstance().getValue("y", Double.class));
            telemetry.addData("orientation", Math.toDegrees(ThreadManager.getInstance().getValue("orientation", Double.class)));
            telemetry.addData("rotation", ThreadManager.getInstance().getValue("rotation", Double.class));
            telemetry.addData("count", ThreadManager.getInstance().getValue("count", Double.class));

            telemetry.addData("er", er.getCurrentPosition());
            telemetry.addData("el", el.getCurrentPosition());
            telemetry.addData("eb", eb.getCurrentPosition());
            telemetry.update();
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