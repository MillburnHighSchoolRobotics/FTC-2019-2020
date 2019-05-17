package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.internal.opmode.OpModeManagerImpl;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

public class Movement {
    DcMotorEx lf;
    DcMotorEx lb;
    DcMotorEx rf;
    DcMotorEx rb;
    double ticksPerRev = 730;
    double wheelRadius = 2;
    public Movement(DcMotor lf, DcMotor lb, DcMotor rf, DcMotor rb) {
        this.lf = (DcMotorEx)lf;
        this.lb = (DcMotorEx)lb;
        this.rf = (DcMotorEx)rf;
        this.rb = (DcMotorEx)rb;
    }
    public void translateDistance(double power, double distance) throws InterruptedException {
        int encoders = (int)(distance/(2*Math.PI*wheelRadius) * ticksPerRev);
        lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        moveToPosition(new DcMotor[] {lf,lb,rf,rb}, new double[] {-power,-power,power,power}, new int[] {(encoders),(encoders),(encoders),(encoders)});
    }
    public void moveToPosition(DcMotor[] motors, double[] power, int[] position) throws InterruptedException {
        for (int x = 0; x < motors.length; x++) {
            motors[x].setPower(power[x]);
            motors[x].setTargetPosition(position[x]);
            motors[x].setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

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
    public static boolean shouldStop() {
        Activity currActivity = AppUtil.getInstance().getActivity();
        OpModeManagerImpl manager = OpModeManagerImpl.getOpModeManagerOfActivity(currActivity);
        OpMode currentOpMode = manager.getActiveOpMode();
        return currentOpMode instanceof LinearOpMode &&
                ((LinearOpMode) currentOpMode).isStarted() &&
                ((LinearOpMode) currentOpMode).isStopRequested();
    }
}