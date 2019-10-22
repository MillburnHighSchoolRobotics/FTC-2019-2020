package org.firstinspires.ftc.teamcode.util;


import android.app.Activity;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.internal.opmode.OpModeManagerImpl;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.threads.ThreadManager;

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
    DcMotorEx lf;
    DcMotorEx lb;
    DcMotorEx rf;
    DcMotorEx rb;
    DcMotorEx ex1;
    DcMotorEx ex2;
    DcMotorEx ey;

    final double p = 0.009;
    final double i = 0.003;
    final double d = 0.0075;

    final double thetaRange = 2;


    public static double ticks = 360*4;
    public static double wheelDiameter = 60/25.4;
    public Movement(DcMotorEx lf, DcMotorEx lb, DcMotorEx rf, DcMotorEx rb, DcMotorEx ex1, DcMotorEx ex2, DcMotorEx ey) {
        this.lf = lf;
        this.lb = lb;
        this.rf = rf;
        this.rb = rb;
        this.ex1 = ex1;
        this.ex2 = ex2;
        this.ey = ey;
    }
    public void translate(double x, double y, double power) throws InterruptedException {
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
        moveWithEncoders(new DcMotor[] {lf,lb,rf,rb}, new DcMotor[] {ex1, ex2, ey}, new double[] {-LF,-LB,RF,RB}, new int[] {distanceToEncoder(x), distanceToEncoder(y)});
    }

    public void rotateTo(double angle) throws InterruptedException {
        ElapsedTime time = new ElapsedTime();
        PIDController pidController = new PIDController(p, i, d, 1, angle);

        double lastTime = -1;

        while (!shouldStop()) {
            if (Thread.currentThread().isInterrupted()) {
                throw new InterruptedException();
            }

            double output = pidController.getPIDOutput(ThreadManager.getInstance().getValue("theta", Double.class));

            if (Math.abs(ThreadManager.getInstance().getValue("theta", Double.class) - angle) < thetaRange) {
                if (lastTime < 0) lastTime = time.milliseconds();
                else if (time.milliseconds() - lastTime > 200) { // ensures the bot is slow when it reaches the theta threshold
                    stop();
                    break;
                }
            } else {
                lastTime = -1;
            }

            lf.setPower(-1 * output);
            lb.setPower(-1 * output);
            rf.setPower(1 * output);
            rb.setPower(1 * output);

            Thread.sleep(5);
        }
    }

    public void moveWithEncoders(DcMotor[] motors, DcMotor[] encoders, double[] power, int[] vector) throws InterruptedException {
        int[] pos1 = new int[] {encoders[0].getCurrentPosition(), encoders[1].getCurrentPosition(), encoders[2].getCurrentPosition()};
        int[] pos2 = new int[] {vector[0]+pos1[0], vector[0]+pos1[1], vector[1]+pos1[2]};

        for (int m = 0; m < motors.length; m++) {
            motors[m].setPower(power[m]);
        }

        while (!shouldStop()) {
            if (Thread.currentThread().isInterrupted()) {
                throw new InterruptedException();
            }
            for (int e = 0; e < encoders.length; e++) {
                if (!encoders[e].isBusy() || (Math.abs(encoders[e].getCurrentPosition()-pos2[e]) < 50)) {
                    break;
                }
            }
            Thread.sleep(10);
        }
        for (DcMotor motor : motors) {
            motor.setPower(0);
        }
    }

    public static int distanceToEncoder(double distance) {
        double circumferenceRaw = Math.PI*wheelDiameter;
        double circumferenceScaled = circumferenceRaw*(48.0/32.0);
        double rotations = distance/circumferenceScaled;
        int encoders = (int)(Math.round(rotations* ticks));
        return encoders;
    }

    public static double encoderToDistance(double encoder) {
        double circumferenceRaw = Math.PI*wheelDiameter;
        double circumferenceScaled = circumferenceRaw*(32.0/48.0);
        double distance = circumferenceScaled * (encoder/ticks);
        return distance; //return inches
    }
    public static double toDegrees(double radians) {
        return radians * (180/Math.PI);
    }
    public void stop() {
        lf.setPower(0);
        lb.setPower(0);
        rf.setPower(0);
        rb.setPower(0);
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