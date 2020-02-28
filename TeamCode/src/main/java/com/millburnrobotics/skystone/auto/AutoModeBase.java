package com.millburnrobotics.skystone.auto;

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
        telemetry.setMsTransmissionInterval(5);
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
            if (time > timer.milliseconds()) {
                break;
            }
            while (periodTimer.milliseconds() < UPDATE_PERIOD);
        }

        if(opModeIsActive() && !isStopRequested()) {
            action.done();
        }
    }
//    public void timeParallelActions(List<Action> actions) { // two simultaneous actions
//        for (Action action : actions) {
//            action.start();
//        }
//
//        double finished = 0;
//        while (finished != actions.size() && opModeIsActive() && !isStopRequested()) {
//            finished = 0;
//            for (Action action : actions) {
//                if (action.isFinished()) {
//                    finished ++;
//                } else {
//                    action.update();
//                    Robot.getInstance().outputToTelemetry(telemetry);
//                    telemetry.update();
//                }
//            }
//            try {
//                Thread.sleep(UPDATE_PERIOD);
//            } catch (InterruptedException e) {
//                e.printStackTrace();
//            }
//        }
//        for (Action action : actions) {
//            action.done();
//        }
//    }
//    public void yParallelAction(Action action1, Action action2, double y) { // one simultaneous action starting after passing *y* on the field
//        boolean currYLess = Robot.getInstance().getOdometry().getPose().y < y;
//        boolean started = false;
//        if(opModeIsActive() && !isStopRequested()) {
//            action1.start();
//        }
//        while (!action1.isFinished() && (opModeIsActive() && !isStopRequested())) {
//            action1.update();
//            if (currYLess) {
//                if (Robot.getInstance().getOdometry().getPose().y > y && !started) {
//                    action2.start();
//                    started = true;
//                }
//            } else {
//                if (Robot.getInstance().getOdometry().getPose().y < y && !started) {
//                    action2.start();
//                    started = true;
//                }
//            }
//            if (started) {
//                action2.update();
//            }
//            Robot.getInstance().outputToTelemetry(telemetry);
//            telemetry.update();
//            try {
//                Thread.sleep(UPDATE_PERIOD);
//            } catch (InterruptedException e) {
//                e.printStackTrace();
//            }
//        }
//    }
//
//    public void timeSeriesActions(Action action1, Action action2, double time) { // one action starting *time* milliseconds after another action
//        ElapsedTime timer = new ElapsedTime();
//        boolean started = false;
//        if(opModeIsActive() && !isStopRequested()) {
//            action1.start();
//        }
//        while (!action1.isFinished() && (opModeIsActive() && !isStopRequested())) {
//            action1.update();
//            if (timer.milliseconds() > time) {
//                action2.start();
//                started = true;
//            }
//            if (started) {
//                action2.update();
//            }
//            Robot.getInstance().outputToTelemetry(telemetry);
//            telemetry.update();
//            try {
//                Thread.sleep(UPDATE_PERIOD);
//            } catch (InterruptedException e) {
//                e.printStackTrace();
//            }
//        }
//
//        if(opModeIsActive() && !isStopRequested()) {
//            action1.done();
//            action2.done();
//        }
//    }
//    public void ySeriesActions(Action action1, Action action2, double y) { // one action starting after passing *y* on the field
//        boolean currYLess = Robot.getInstance().getOdometry().getPose().y < y;
//        boolean started = false;
//        boolean done = false;
//        if(opModeIsActive() && !isStopRequested()) {
//            action1.start();
//        }
//        while (!done && (opModeIsActive() && !isStopRequested())) {
//            action1.update();
//            if (currYLess) {
//                if (Robot.getInstance().getOdometry().getPose().y > y && !started) {
//                    action2.done();
//                    action2.start();
//                    started = true;
//                }
//            } else {
//                if (Robot.getInstance().getOdometry().getPose().y < y && !started) {
//                    action1.done();
//                    action2.start();
//                    started = true;
//                }
//            }
//            if (started) {
//                action2.update();
//                done = action2.isFinished();
//            }
//            Robot.getInstance().outputToTelemetry(telemetry);
//            telemetry.update();
//            try {
//                Thread.sleep(UPDATE_PERIOD);
//            } catch (InterruptedException e) {
//                e.printStackTrace();
//            }
//        }
//
//        if(opModeIsActive() && !isStopRequested()) {
//            action2.done();
//        }
//    }
}
