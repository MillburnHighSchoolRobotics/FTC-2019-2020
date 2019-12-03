package org.firstinspires.ftc.teamcode.test;

import android.app.Activity;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.internal.opmode.OpModeManagerImpl;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.threads.PositionMonitor;
import org.firstinspires.ftc.teamcode.threads.ThreadManager;

@Autonomous(group = "test")
public class FPSTest extends LinearOpMode {

    DcMotorEx er;
    DcMotorEx el;
    DcMotorEx eb;

    @Override
    public void runOpMode() throws InterruptedException {
        er = (DcMotorEx) hardwareMap.dcMotor.get("er");
        el = (DcMotorEx) hardwareMap.dcMotor.get("el");
        eb = (DcMotorEx) hardwareMap.dcMotor.get("eb");
        er.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        er.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        el.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        el.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        eb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        eb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        ThreadManager manager = ThreadManager.getInstance();
        manager.setHardwareMap(hardwareMap);
        manager.setupThread("PositionMonitor", PositionMonitor.class, new Pose2d());
        waitForStart();

        while (!shouldStop()) {
            telemetry.addData("theta", Math.toDegrees(ThreadManager.getInstance().getValue("theta", Double.class)));
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