package org.firstinspires.ftc.teamcode.roadrunner_old.MotionProfiling;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.tuning.AccelRegression;
import com.acmerobotics.roadrunner.tuning.RampRegression;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.internal.system.Misc;
import org.firstinspires.ftc.teamcode.roadrunner_old.DriveBase;
import org.firstinspires.ftc.teamcode.roadrunner_old.MohanBot;
import org.firstinspires.ftc.teamcode.threads.PositionMonitor;
import org.firstinspires.ftc.teamcode.threads.ThreadManager;

import static org.firstinspires.ftc.teamcode.roadrunner_old.DriveConstants.getMaxRpm;
import static org.firstinspires.ftc.teamcode.roadrunner_old.DriveConstants.rpmToVelocity;

/*
 * Op mode for computing kV, kStatic, and kA from various drive routines. For the curious, here's an
 * outline of the procedure:
 *   1. Slowly ramp the motor power and record encoder values along the way.
 *   2. Run a linear regression on the encoder velocity vs. motor power plot to obtain a slope (kV)
 *      and an optional intercept (kStatic).
 *   3. Accelerate the robot (apply constant power) and record the encoder counts.
 *   4. Adjust the encoder data based on the velocity tuning data and find kA with another linear
 *      regression.
 */
@Autonomous(group = "drive")
public class DriveFeedforwardTuner extends LinearOpMode {
    public static final double MAX_POWER = 0.7;
    public static final double DISTANCE = 91;

    @Override
    public void runOpMode() throws InterruptedException {
        ThreadManager manager = ThreadManager.getInstance();
        manager.setHardwareMap(hardwareMap);
        manager.setupThread("PositionMonitor", PositionMonitor.class);
        DriveBase drive = new MohanBot(hardwareMap);

        NanoClock clock = NanoClock.system();

        telemetry.log().add("Press play to begin the feedforward tuning routine");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        telemetry.log().clear();
        telemetry.log().add("Would you like to fit kStatic?");
        telemetry.log().add("Press (A) for yes, (B) for no");
        telemetry.update();

        boolean fitIntercept = false;
        while (!isStopRequested()) {
            if (gamepad1.a) {
                fitIntercept = true;
                while (!isStopRequested() && gamepad1.a) {
                    idle();
                }
                break;
            } else if (gamepad1.b) {
                while (!isStopRequested() && gamepad1.b) {
                    idle();
                }
                break;
            }
            idle();
        }

        telemetry.log().clear();
        telemetry.log().add(Misc.formatInvariant(
                "Place your robot on the field with at least %.2f in of room in front", DISTANCE));

        double maxVel = rpmToVelocity(getMaxRpm());
        double finalVel = MAX_POWER * maxVel;
        double accel = (finalVel * finalVel) / (2.0 * DISTANCE);
        double rampTime = Math.sqrt(2.0 * DISTANCE / accel);

        telemetry.log().add(Misc.formatInvariant(
                "Max Vel: %.2f", maxVel));
        telemetry.log().add(Misc.formatInvariant(
                "Final Vel: %.2f", finalVel));
        telemetry.log().add(Misc.formatInvariant(
                "Accel: %.2f", accel));
        telemetry.log().add(Misc.formatInvariant(
                "Ramp Time: %.2f", rampTime));

        telemetry.log().add("Press (A) to begin");
        telemetry.update();

        while (!isStopRequested() && !gamepad1.a) {
            idle();
        }
        while (!isStopRequested() && gamepad1.a) {
            idle();
        }

        telemetry.log().clear();
        telemetry.log().add("Running...");
        telemetry.update();

        double startTime = clock.seconds();
        RampRegression rampRegression = new RampRegression();

        drive.setPoseEstimate(new Pose2d());
        while (!isStopRequested()) {
            double elapsedTime = clock.seconds() - startTime;
            if (elapsedTime > rampTime) {
//                telemetry.log().add("break");
                break;
            }
            double vel = accel * elapsedTime;
            double power = vel / maxVel;

            double driveX = drive.getPoseEstimate().getX();

//            telemetry.log().add("Vel: " + vel + "\tPower: " + (vel/rpmToVelocity(getMaxRpm())) + "\tX: " + driveX);
//            telemetry.update();

            rampRegression.add(elapsedTime, driveX, power);

            drive.setDrivePower(new Pose2d(power, 0.0, 0.0));
            drive.updatePoseEstimate();
        }
        drive.setDrivePower(new Pose2d(0.0, 0.0, 0.0));

        RampRegression.RampResult rampResult = rampRegression.fit(fitIntercept);


//        telemetry.log().clear();
//        drive.updatePoseEstimate();
//        telemetry.log().add(String.valueOf(drive.getPoseEstimate().getX()));
        telemetry.log().add("Quasi-static ramp up test complete");
        drive.updatePoseEstimate();
        telemetry.log().add(String.valueOf(drive.getPoseEstimate().getX()));

        if (fitIntercept) {
            telemetry.log().add(Misc.formatInvariant("kV = %.5f, kStatic = %.5f (R^2 = %.2f)",
                    rampResult.kV, rampResult.kStatic, rampResult.rSquare));
        } else {
            telemetry.log().add(Misc.formatInvariant("kV = %.5f (R^2 = %.2f)",
                    rampResult.kStatic, rampResult.rSquare));
        }
        telemetry.log().add("Would you like to fit kA?");
        telemetry.log().add("Press (A) for yes, (B) for no");
        telemetry.update();

        boolean fitAccelFF = false;
        while (!isStopRequested()) {
            if (gamepad1.a) {
                fitAccelFF = true;
                while (!isStopRequested() && gamepad1.a) {
                    idle();
                }
                break;
            } else if (gamepad1.b) {
                while (!isStopRequested() && gamepad1.b) {
                    idle();
                }
                break;
            }
            idle();
        }

        if (fitAccelFF) {
            telemetry.log().clear();
            telemetry.log().add("Place the robot back in its starting position");
            telemetry.log().add("Press (A) to continue");
            telemetry.update();

            while (!isStopRequested() && !gamepad1.a) {
                idle();
            }
            while (!isStopRequested() && gamepad1.a) {
                idle();
            }

            telemetry.log().clear();
            telemetry.log().add("Running...");
            telemetry.update();

            double maxPowerTime = DISTANCE / maxVel;

            startTime = clock.seconds();
            AccelRegression accelRegression = new AccelRegression();

            drive.setPoseEstimate(new Pose2d());
            drive.setDrivePower(new Pose2d(MAX_POWER, 0.0, 0.0));
            while (!isStopRequested()) {
                double elapsedTime = clock.seconds() - startTime;
                if (elapsedTime > maxPowerTime) {
                    break;
                }

                accelRegression.add(elapsedTime, drive.getPoseEstimate().getX(), MAX_POWER);

                drive.updatePoseEstimate();
            }
            drive.setDrivePower(new Pose2d(0.0, 0.0, 0.0));

            AccelRegression.AccelResult accelResult = accelRegression.fit(
                    rampResult.kV, rampResult.kStatic);

            telemetry.log().clear();
            telemetry.log().add("Constant power test complete");
            telemetry.log().add(Misc.formatInvariant("kA = %.5f (R^2 = %.2f)",
                    accelResult.kA, accelResult.rSquare));
            telemetry.update();
        }

        while (!isStopRequested()) {
            idle();
        }
    }
}