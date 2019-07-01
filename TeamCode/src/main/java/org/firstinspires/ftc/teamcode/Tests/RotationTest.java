package org.firstinspires.ftc.teamcode.Tests;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.RobotControl.Movement;
import org.firstinspires.ftc.teamcode.ThreadManager.PositionMonitor;
import org.firstinspires.ftc.teamcode.ThreadManager.ThreadManager;

@Autonomous(name = "Rotation Test", group = "test")
public class RotationTest extends LinearOpMode {

    DcMotorEx lf;
    DcMotorEx lb;
    DcMotorEx rf;
    DcMotorEx rb;
    DcMotorEx ex1;
    DcMotorEx ex2;
    DcMotorEx ey;

    @Override
    public void runOpMode() throws InterruptedException {
        lf = (DcMotorEx) hardwareMap.dcMotor.get("lf");
        lb = (DcMotorEx) hardwareMap.dcMotor.get("lb");
        rf = (DcMotorEx) hardwareMap.dcMotor.get("rf");
        rb = (DcMotorEx) hardwareMap.dcMotor.get("rb");
        ex1 = (DcMotorEx) hardwareMap.dcMotor.get("lf");
        ex2 = (DcMotorEx) hardwareMap.dcMotor.get("lb");
        ey = (DcMotorEx) hardwareMap.dcMotor.get("rf");
        lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ex1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ex1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ex2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ex2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ey.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ey.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Movement mv = new Movement(lf,lb,rf,rb,ex1,ex2,ey);
        ThreadManager manager = ThreadManager.getInstance();
        manager.setHardwareMap(hardwareMap);
        manager.setupThread("PositionMonitor", PositionMonitor.class);
        waitForStart();

        while (true) {
            telemetry.addData("PositionMonitor", ThreadManager.getInstance().getValue("rotation", Float.class));
            telemetry.update();
        }
    }
}