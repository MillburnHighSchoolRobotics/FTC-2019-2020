package com.millburnrobotics.skystone.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.millburnrobotics.skystone.Constants;
import com.millburnrobotics.skystone.Robot;
import com.millburnrobotics.skystone.auto.actions.Action;
import com.millburnrobotics.skystone.auto.actions.RobotUpdateAction;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import static com.millburnrobotics.skystone.Constants.UPDATE_PERIOD;

public abstract class AutoModeBase extends LinearOpMode {

    protected ElapsedTime autoTimer = new ElapsedTime();

    public abstract void routine();

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(25);
        telemetry.addData("startup", "Loading...");
        telemetry.update();
        Robot.getInstance().init(hardwareMap, true);
        telemetry.addData("startup", "Loaded!");
        telemetry.update();
        waitForStart();

        if (opModeIsActive() && !isStopRequested()) {
            autoTimer.reset();
            threadAction(new RobotUpdateAction(autoTimer));
            routine();
        }
    }
    public void runAction(Action action) {
        runAction(action, UPDATE_PERIOD);
    }
    public void runAction(Action action, long period) { // single complete action
        ElapsedTime periodTimer = new ElapsedTime();
        if(opModeIsActive() && !isStopRequested()) {
            action.start();
        }
        while (!action.isFinished() && (opModeIsActive() && !isStopRequested())) {
            periodTimer.reset();
            action.update();
            Robot.getInstance().outputToTelemetry(telemetry);
            telemetry.update();

            while (periodTimer.milliseconds() < period);
        }

        if(opModeIsActive() && !isStopRequested()) {
            action.done();
        }
    }

    public void threadAction(final Action action) { // action on separate thread
        Runnable runnable = () -> runAction(action, Constants.FPS_UPDATE_PERIOD);

        if(opModeIsActive() && !isStopRequested())
            new Thread(runnable).start();
    }

    public void timedAction(Action action, double time) { // action that stops after *time* milliseconds
        ElapsedTime periodTimer = new ElapsedTime();
        ElapsedTime timer = new ElapsedTime();
        if(opModeIsActive() && !isStopRequested()) {
            action.start();
        }
        while (!action.isFinished() && (opModeIsActive() && !isStopRequested())) {
            periodTimer.reset();
            action.update();
            Robot.getInstance().outputToTelemetry(telemetry);
            telemetry.update();
            if (time < timer.milliseconds()) {
                break;
            }
            while (periodTimer.milliseconds() < UPDATE_PERIOD);
        }

        if(opModeIsActive() && !isStopRequested()) {
            action.done();
        }
    }
}
