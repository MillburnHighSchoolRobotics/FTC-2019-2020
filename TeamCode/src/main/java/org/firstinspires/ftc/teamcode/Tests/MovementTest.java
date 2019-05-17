package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Movement;

@Autonomous(name = "Movement Test", group = "test")
public class MovementTest extends LinearOpMode {

    DcMotor motor;
    DcMotor encoder;
//    DcMotor lf;
//    DcMotor lb;
//    DcMotor rf;
//    DcMotor rb;
//
    @Override
    public void runOpMode() throws InterruptedException {
        motor = hardwareMap.dcMotor.get("motor");
        encoder = hardwareMap.dcMotor.get("motor");

        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setPower(730);
        motor.setTargetPosition(1000);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (!shouldStop()) {
            if (Thread.currentThread().isInterrupted()) {
                throw new InterruptedException();
            }
            for (int x = 0; x < motors.length; x++) {
                if (!motors[x].isBusy() || (Math.abs(motors[x].getCurrentPosition()-position[x]) < 50)) {
                    break;
                }
            }
            Thread.sleep(10);
        }
        for (DcMotor motor : motors) {
            motor.setPower(0);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
//        lf = hardwareMap.dcMotor.get("leftFront");
//        lb = hardwareMap.dcMotor.get("leftBack");
//        rf = hardwareMap.dcMotor.get("rightFront");
//        rb = hardwareMap.dcMotor.get("rightBack");
//
//        rf.setDirection(DcMotorSimple.Direction.REVERSE);
//        rb.setDirection(DcMotorSimple.Direction.REVERSE);
//        lf.setDirection(DcMotorSimple.Direction.FORWARD);
//        lb.setDirection(DcMotorSimple.Direction.FORWARD);
//
//        Movement mv = new Movement(lf, lb, rf, rb);
//
//        mv.translateDistance(0.7, 10);
    }
}