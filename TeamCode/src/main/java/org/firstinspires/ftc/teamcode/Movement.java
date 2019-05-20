package org.firstinspires.ftc.teamcode;


import android.app.Activity;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.internal.opmode.OpModeManagerImpl;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

public class Movement {
    public static final int[][] POWER_MATRIX = { //for each of the directions
            {1, 1, 1, 1},
            {1, 0, 0, 1},
            {1, -1, -1, 1},
            {0, -1, -1, 0},
            {-1, -1, -1, -1},
            {-1, 0, 0, -1},
            {-1, 1, 1, -1},
            {0, 1, 1, 0}
    };
    DcMotorEx lf; //motors
    DcMotorEx lb;
    DcMotorEx rf;
    DcMotorEx rb;
    DcMotorEx x1; //encoders
    DcMotorEx x2;
    DcMotorEx y1;
    public static double ticksPerRev = 1120;
    public static double wheelRadius = 2;
    public Movement(DcMotor lf, DcMotor lb, DcMotor rf, DcMotor rb, DcMotor x1, DcMotor x2, DcMotor y1) {
        this.lf = (DcMotorEx)lf;
        this.lb = (DcMotorEx)lb;
        this.rf = (DcMotorEx)rf;
        this.rb = (DcMotorEx)rb;
        this.x1 = (DcMotorEx)x1;
        this.x2 = (DcMotorEx)x2;
        this.y1 = (DcMotorEx)y1;
    }
    public void translate(double x, double y, double power) throws InterruptedException {
        double distance = Math.sqrt(x*x + y*y);
        double theta = Math.atan(x/y);

        double scale;
        if (theta < 0) theta += 360;
        double RF = 0, RB = 0, LF = 0, LB = 0;
        if (theta >= 0 && theta <= 90) { //quadrant 1
            scale = Math.sin(Math.toRadians(theta - 45)) / Math.cos(Math.toRadians(theta - 45));
            LF = power * POWER_MATRIX[0][0];
            LB = power * POWER_MATRIX[0][1] * scale;
            RF = power * POWER_MATRIX[0][2] * scale;
            RB = power * POWER_MATRIX[0][3];
        } else if (theta > 90 && theta <= 180) { //quadrant 2
            power *= -1;
            scale = Math.sin(Math.toRadians(theta - 135)) / Math.cos(Math.toRadians(theta - 135));
            LF = (power * POWER_MATRIX[2][0] * scale);
            LB = (power * POWER_MATRIX[2][1]);
            RF = (power * POWER_MATRIX[2][2]);
            RB = (power * POWER_MATRIX[2][3] * scale);
        } else if (theta > 180 && theta <= 270) { //quadrant 3
            scale = Math.sin(Math.toRadians(theta - 225)) / Math.cos(Math.toRadians(theta -
                    225));
            LF = (power * POWER_MATRIX[4][0]);
            LB = (power * POWER_MATRIX[4][1] * scale);
            RF = (power * POWER_MATRIX[4][2] * scale);
            RB = (power * POWER_MATRIX[4][3]);
        } else if (theta > 270 && theta < 360) { //quadrant 4
            power *= -1;
            scale = Math.sin(Math.toRadians(theta - 315)) / Math.cos(Math.toRadians(theta - 315));
            LF = (power * POWER_MATRIX[6][0] * scale);
            LB = (power * POWER_MATRIX[6][1]);
            RF = (power * POWER_MATRIX[6][2]);
            RB = (power * POWER_MATRIX[6][3] * scale);
        }
        //int encoder = ;
        x1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        x2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        y1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        moveToPosition(new DcMotor[] {lf,lb,rf,rb}, new DcMotor[] {x1, x2, y1}, new double[] {-LF,-LB,RF,RB}, new int[] {distanceToEncoder(x), distanceToEncoder(y)});
    }
    public void moveToPosition(DcMotor[] motors, DcMotor[] encoders, double[] power, int[] vector) throws InterruptedException {



        encoders[0].setTargetPosition(vector[0]);
        encoders[1].setTargetPosition(vector[0]);
        encoders[2].setTargetPosition(vector[1]);
        for (int x = 0; x < motors.length; x++) {
            motors[x].setPower(power[x]);
            motors[x].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            encoders[x].setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        while (!shouldStop()) {
            if (Thread.currentThread().isInterrupted()) {
                throw new InterruptedException();
            }
            for (int x = 0; x < motors.length; x++) {
                if (!encoders[x].isBusy() || (Math.abs(encoders[x].getCurrentPosition()-vector[x]) < 50)) {
                    break;
                }
            }
            Thread.sleep(10);
        }
        for (DcMotor motor : motors) {
            motor.setPower(0);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    public static int distanceToEncoder(double distance) {
        return (int)(distance/(2*Math.PI*wheelRadius) * ticksPerRev);
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