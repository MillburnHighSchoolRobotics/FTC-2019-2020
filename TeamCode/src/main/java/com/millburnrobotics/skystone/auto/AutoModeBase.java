package com.millburnrobotics.skystone.auto;

import com.millburnrobotics.skystone.Constants;
import com.millburnrobotics.skystone.auto.actions.Action;
import com.millburnrobotics.skystone.auto.actions.RobotUpdateAction;
import com.millburnrobotics.skystone.Robot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;

import static com.millburnrobotics.skystone.Constants.UPDATE_PERIOD;

public abstract class AutoModeBase extends LinearOpMode {

    protected ElapsedTime autoTimer = new ElapsedTime();

    public abstract void routine();

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.setMsTransmissionInterval(5);
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
        if(opModeIsActive() && !isStopRequested()) {
            action.start();
        }
        while (!action.isFinished() && (opModeIsActive() && !isStopRequested())) {
            action.update();
            Robot.getInstance().outputToTelemetry(telemetry);
            telemetry.update();
            try {
                Thread.sleep(period);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }

        if(opModeIsActive() && !isStopRequested()) {
            action.done();
        }
    }

    public void threadAction(final Action action) { // action on separate thread
        Runnable runnable = () -> runAction(action, Constants.OdometryConstants.FPS_UPDATE_PERIOD);

        if(opModeIsActive() && !isStopRequested())
            new Thread(runnable).start();
    }

    public void timedAction(Action action, double time) { // action that stops after *time* milliseconds
        ElapsedTime timer = new ElapsedTime();
        if(opModeIsActive() && !isStopRequested()) {
            action.start();
        }
        while ((timer.milliseconds() > time) || (!action.isFinished() && (opModeIsActive() && !isStopRequested()))) {
            action.update();
            Robot.getInstance().outputToTelemetry(telemetry);
            telemetry.update();
            try {
                Thread.sleep(UPDATE_PERIOD);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }

        if(opModeIsActive() && !isStopRequested()) {
            action.done();
        }
    }
    public void parallelActions(List<Action> actions) { // two simultaneous actions
        for (Action action : actions) {
            action.start();
        }

        double finished = 0;
        while (finished != actions.size() && opModeIsActive() && !isStopRequested()) {
            finished = 0;
            for (Action action : actions) {
                if (action.isFinished()) {
                    finished ++;
                } else {
                    action.update();
                    Robot.getInstance().outputToTelemetry(telemetry);
                    telemetry.update();
                }
            }
            try {
                Thread.sleep(UPDATE_PERIOD);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
        for (Action action : actions) {
            action.done();
        }
    }

    public void seriesActions(Action action1, Action action2, double time) { // one action starting *time* milliseconds after another action
        ElapsedTime timer = new ElapsedTime();
        boolean started = false;
        if(opModeIsActive() && !isStopRequested()) {
            action1.start();
        }
        while (!action1.isFinished() && (opModeIsActive() && !isStopRequested())) {
            action1.update();
            if (timer.milliseconds() > time) {
                action2.start();
                started = true;
            }
            if (started) {
                action2.update();
            }
            Robot.getInstance().outputToTelemetry(telemetry);
            telemetry.update();
            try {
                Thread.sleep(UPDATE_PERIOD);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }

        if(opModeIsActive() && !isStopRequested()) {
            action1.done();
            action2.done();
        }
    }
}
