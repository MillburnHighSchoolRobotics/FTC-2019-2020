package com.millburnrobotics.skystone.subsystems;

import android.util.Log;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.millburnrobotics.lib.control.Path;
import com.millburnrobotics.lib.geometry.Pose;
import com.millburnrobotics.lib.profile.MotionState;
import com.millburnrobotics.lib.util.MathUtils;
import com.millburnrobotics.lib.util.PIDController;
import com.millburnrobotics.skystone.Robot;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Arrays;
import java.util.Collections;
import java.util.List;

import static com.millburnrobotics.skystone.Constants.DriveConstants.LOOK_AHEAD;
import static com.millburnrobotics.skystone.Constants.DriveConstants.MAX_A;
import static com.millburnrobotics.skystone.Constants.DriveConstants.MAX_V;
import static com.millburnrobotics.skystone.Constants.DriveConstants.ROTATION_THRESHOLD;
import static com.millburnrobotics.skystone.Constants.IMUConstants.COLLISION_RECOVERY_TIME;

public class Drive extends Subsystem {
    public enum DriveState {
        FIELD_CENTRIC,
        ROBOT_CENTRIC
    }
    private DriveState state;

    PIDController headingController = new PIDController(0.014,0.002,0.004);
    public double kv = 0.013069853913931;
    public double ka = 0.00013617775604739;
    public double ks = 0.11659998299699;
    public double kp = 0.033;

    private Pose lookahead = new Pose();
    private double ffpower;

    private ElapsedTime changeStateTimer = new ElapsedTime();

    @Override
    public void init(boolean auto) {
        stop();
        this.state = DriveState.ROBOT_CENTRIC;
    }

    @Override
    public void outputToTelemetry(Telemetry telemetry, TelemetryPacket packet) {
        telemetry.addData("DriveState", state.name());
        /*
        public String toString() {
        return String.format(
                "\n" +
                "(%.1f)---(%.1f)\n" +
                "|   Front   |\n" +
                "|           |\n" +
                "|           |\n" +
                "(%.1f)---(%.1f)\n"
                , frontLeft, frontRight, backLeft, backRight);
    }
         */
    }


    @Override
    public void update() {

    }
    public void setDrivePower(double lfPower, double lbPower, double rfPower, double rbPower) {
        Robot.getInstance().lf.setPower(lfPower);
        Robot.getInstance().lb.setPower(lbPower);
        Robot.getInstance().rf.setPower(-rfPower);
        Robot.getInstance().rb.setPower(-rbPower);
    }
    public void setDrivePower(double[] powers) {
        Log.d("motorpowers", Arrays.toString(powers));
        setDrivePower(powers[0],powers[1],powers[2],powers[3]);
    }
    public void setDrivePower(double power) {
        setDrivePower(power,power,power,power);
    }
    public void setDrivePower(double powerLeft, double powerRight) {
        setDrivePower(powerLeft, powerLeft, powerRight, powerRight);
    }
    public void setDrivePower(Pose p) {
        setDrivePower(p.x,p.y,p.heading);
    }
    public void setDrivePower(double x, double y, double turnPower) {
        setDrivePower(scale(new double[]{
                y + turnPower + x,
                y + turnPower + x,
                y - turnPower - x,
                y - turnPower - x
        }));
    }
    public void stop() {
        setDrivePower(0);
    }
    public void rotateTo(double target, double power) {
        double targetHeading = Math.toDegrees(MathUtils.normalize(Math.toRadians(target)));
        headingController.setTarget(targetHeading);

        double currentHeading = Math.toDegrees(Robot.getInstance().getOdometry().getPose().getHeading());
        if (currentHeading - targetHeading > 180) {
            currentHeading -= 360;
        } else if (targetHeading - currentHeading > 180) {
            currentHeading += 360;
        }
        double output = headingController.getPIDOutput(currentHeading);
        Log.d("turn pid", "output - " + output);
        Log.d("turn pid", "current heading - " + currentHeading);
        Log.d("turn pid", "target heading - " + targetHeading);
        setDrivePower(-power*output,power*output);
    }
    public void vectorTo(Pose currentPose, Pose targetPose, double power) {
        setDrivePower(powerVector(currentPose,targetPose,power));
    }
    public void vectorTo(Pose currentPose, Pose targetPose, double power, double rotationPower) {
        double[] motorPowers = {0,0,0,0};

        if (!MathUtils.equals(currentPose.distTo(targetPose), 0, 0.05)) {
            motorPowers = powerVector(currentPose, targetPose, Math.pow(Math.abs(currentPose.distTo(targetPose)),(9.0/7.0)));
        }
        motorPowers[0] += rotationPower;
        motorPowers[1] += rotationPower;
        motorPowers[2] -= rotationPower;
        motorPowers[3] -= rotationPower;

        motorPowers = scalePowerArray(motorPowers, 0);
        for (int x = 0; x < motorPowers.length; x++) {
            motorPowers[x] = motorPowers[x]*power;
        }
        setDrivePower(motorPowers);
    }
    private double[] scalePowerArray(double[] motorPowers, double minPower) {
        double maxArrayPower = MathUtils.maxArray(motorPowers);
        if (maxArrayPower < 1) {
            maxArrayPower = 1;
        }
        for (int x = 0; x < motorPowers.length; x++) {
            motorPowers[x] = MathUtils.sgn(motorPowers[x]) * MathUtils.map(Math.abs(motorPowers[x]),0,maxArrayPower,minPower,1);
        }
        return motorPowers;
    }
    private double[] scale(double[] motorPowers) {
        double absMax = MathUtils.maxArray(motorPowers);
        if (absMax > 1) {
            motorPowers[0] /= absMax;
            motorPowers[1] /= absMax;
            motorPowers[2] /= absMax;
            motorPowers[3] /= absMax;
        }
        return motorPowers;
    }
    private double[] powerVector(Pose currentPose, Pose targetPose, double power) {
        double scale, lf = 0, lb = 0, rf = 0, rb = 0;
        double absoluteAngle = Math.atan2(targetPose.getY() - currentPose.getY(), targetPose.getX() - currentPose.getX());
        double relAngle = Math.toDegrees(MathUtils.normalize(absoluteAngle - currentPose.getHeading()));

        if (relAngle >= 0 && relAngle < 90) {
            scale = MathUtils.tanDegrees(relAngle - 45);
            lf = 1;
            lb = scale;
            rf = scale;
            rb = 1;
        } else if (relAngle >= 90 && relAngle < 180) {
            scale = MathUtils.tanDegrees(relAngle - 135);
            lf = -scale;
            lb = 1;
            rf = 1;
            rb = -scale;
        } else if (relAngle >= 180 && relAngle < 270) {
            scale = MathUtils.tanDegrees(relAngle - 225);
            lf = -1;
            lb = -scale;
            rf = -scale;
            rb = -1;
        } else if (relAngle >= 270 && relAngle < 360) {
            scale = MathUtils.tanDegrees(relAngle - 315);
            lf = scale;
            lb = -1;
            rf = -1;
            rb = scale;
        }
        lf *= power;
        lb *= power;
        rf *= power;
        rb *= power;


        double currentHeading = Math.toDegrees(currentPose.getHeading());
        double targetHeading = Math.toDegrees(targetPose.getHeading());
        if (currentHeading - targetHeading > 180) {
            currentHeading -= 360;
        } else if (targetHeading - currentHeading > 180) {
            currentHeading += 360;
        }
        double u = headingController.getKp()*(targetHeading-currentHeading);
        if (MathUtils.equals(targetHeading, currentHeading, ROTATION_THRESHOLD)) {
            u = 0;
        }

        lf -= (1-power)*u;
        lb -= (1-power)*u;
        rf += (1-power)*u;
        rb += (1-power)*u;

        return new double[] {lf,lb,rf,rb};
    }
    public void updatePathFollower(Pose currentPose, Pose currentVelocity, Path path) {
        path.update(currentPose);
        Pose nextPose = path.nextPose(currentPose);
        this.lookahead = nextPose;
        if (currentPose.distTo(path.end()) > 18) {
//            Pose relPPTarget = currentPose.relDistanceToTarget(nextPose);
//            Log.d("relPPTarget", relPPTarget+"");
//            Pose translationPowers = relPPTarget.times(-1).div(new Pose(LOOK_AHEAD, LOOK_AHEAD, Math.PI));
//            double angleToTarget = MathUtils.normalize(nextPose.heading - currentPose.heading);
//            translationPowers.heading = angleToTarget / Math.PI;

            double power;
            double dist = currentPose.distTo(path.end());
            if (Math.pow(currentVelocity.norm(),2)/(2*MAX_A) > dist) { // slow down
                power = kv*Math.sqrt(2*MAX_A*dist)+ka*MAX_A;
                power += (MathUtils.sgn(power) * ks);

            } else { // max
                power = kv*MAX_V;
                power += (MathUtils.sgn(power) * ks);
            }
//            Log.d("desmosplshelp", "(x-" + Robot.getInstance().timer.milliseconds()/100.0 + ")^{2}+(y-" + motionState.v + ")^{2}=0.2");
//            Log.d("desmosplshelp", "(x-" + Robot.getInstance().timer.milliseconds()/100.0 + ")^{2}+(y-" + currentVelocity.norm() + ")^{2}=0.05");

            if (Robot.getInstance().getIMU().collided()) {
                vectorTo(nextPose, currentPose, power);
                ElapsedTime collisionWait = new ElapsedTime();
                while (collisionWait.milliseconds() < COLLISION_RECOVERY_TIME);
            } else {
                vectorTo(currentPose, nextPose, power);
            }
        } else {
            vectorTo(currentPose,path.end(),0.2);
//            Pose relAbsTarget = currentPose.relDistanceToTarget(path.end());
//            double angleToTarget = path.end().minus(currentPose).normalize();
//            double exp = 1.0/6.0;
//            Pose dirPowers = new Pose(
//                    -MathUtils.powRetainingSign(relAbsTarget.x, exp),
//                    -MathUtils.powRetainingSign(relAbsTarget.y, exp),
//                    MathUtils.powRetainingSign(angleToTarget, exp)
//            );
//            setDrivePower(dirPowers.times(new Pose(0.08,0.12,0.1)));
        }

//        System.out.println(translationPowers.toString());

        // Heading always wants to stop at a point, so we'll treat this the same regardless if we're
        // at a stop waypoint or a normal one
//        double forwardAngle = nextPose.minus(currentPose).atan();
//        double backwardAngle = forwardAngle + Math.PI;
//        double angleToForward = MathUtils.normalize(forwardAngle - currentPose.heading);
//        double angleToBackward = MathUtils.normalize(backwardAngle - currentPose.heading);
//        double autoAngle = Math.abs(angleToForward) < Math.abs(angleToBackward) ? forwardAngle : backwardAngle;
//        double desiredAngle = nextPose.heading;
    }
    public Pose getLookahead() {
        return lookahead;
    }
    public double getFFpower() {
        return ffpower;
    }
    public void setState(DriveState state) {
        if (changeStateTimer.milliseconds() > 250) {
            this.state = state;
            changeStateTimer.reset();
        }
    }
    public DriveState getState() {
        return state;
    }
}