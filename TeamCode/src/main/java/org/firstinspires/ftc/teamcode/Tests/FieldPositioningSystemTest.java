package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.ThreadManager.PositionMonitor;
import org.firstinspires.ftc.teamcode.ThreadManager.ThreadManager;

@Autonomous(name = "FPS Test", group = "test")
public class FieldPositioningSystemTest extends LinearOpMode {

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
        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
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

        ThreadManager manager = ThreadManager.getInstance();
        manager.setHardwareMap(hardwareMap);
        manager.setupThread("PositionMonitor", PositionMonitor.class);
        waitForStart();

        while (true) {
            telemetry.addData("theta", ThreadManager.getInstance().getValue("theta", Double.class));
            telemetry.addData("x", ThreadManager.getInstance().getValue("x", Double.class));
            telemetry.addData("y", ThreadManager.getInstance().getValue("y", Double.class));
            telemetry.addData("rotation", ThreadManager.getInstance().getValue("rotation", Integer.class));
            telemetry.addData("ex1", ex1.getCurrentPosition());
            telemetry.addData("ex2", ex2.getCurrentPosition());
            telemetry.addData("ey", ey.getCurrentPosition());
            telemetry.update();
        }
    }
}