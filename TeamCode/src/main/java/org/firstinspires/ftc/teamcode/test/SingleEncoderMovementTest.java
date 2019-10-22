package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import static org.firstinspires.ftc.teamcode.util.Movement.shouldStop;
import static org.firstinspires.ftc.teamcode.util.Movement.ticks;
import static org.firstinspires.ftc.teamcode.util.Movement.wheelDiameter;

@Autonomous(group = "test")
public class SingleEncoderMovementTest extends LinearOpMode {

    DcMotorEx lf;
    DcMotorEx lb;
    DcMotorEx rf;
    DcMotorEx rb;
    DcMotorEx ex1;

    @Override
    public void runOpMode() throws InterruptedException {
        lf = (DcMotorEx) hardwareMap.dcMotor.get("lf");
        lb = (DcMotorEx) hardwareMap.dcMotor.get("lb");
        rf = (DcMotorEx) hardwareMap.dcMotor.get("rf");
        rb = (DcMotorEx) hardwareMap.dcMotor.get("rb");
        ex1 = (DcMotorEx) hardwareMap.dcMotor.get("lb");
        lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ex1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ex1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        waitForStart();

        int distance = 12;
        int encoders = (int)(Math.round((distance/((Math.PI*wheelDiameter)*(48.0/32.0)))*ticks));

        int pos1 = ex1.getCurrentPosition();
        int pos2 = pos1+encoders;

        ex1.setTargetPosition(pos2);
        lf.setPower(0.8);
        lb.setPower(0.8);
        rf.setPower(0.8);
        rb.setPower(0.8);

        while (!shouldStop()) {
            if (Thread.currentThread().isInterrupted()) {
                throw new InterruptedException();
            }
            if (!ex1.isBusy() || (Math.abs(ex1.getCurrentPosition()-pos2) < 50)) {
                break;
            }
            Thread.sleep(10);
        }
        lf.setPower(0);
        lb.setPower(0);
        rf.setPower(0);
        rb.setPower(0);
    }
}