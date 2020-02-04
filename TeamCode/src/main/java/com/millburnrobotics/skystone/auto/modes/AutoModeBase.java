package com.millburnrobotics.skystone.auto.modes;

import com.millburnrobotics.skystone.auto.actions.Action;
import com.millburnrobotics.skystone.auto.actions.RobotUpdateAction;
import com.millburnrobotics.skystone.subsystems.Robot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;

public abstract class AutoModeBase extends LinearOpMode {

    protected ElapsedTime autoTimer = new ElapsedTime();

    public abstract void routine();

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        autoTimer.reset();

        Robot.getInstance().init(hardwareMap, true);
        ThreadAction(new RobotUpdateAction(autoTimer));

        if (opModeIsActive() && !isStopRequested()) {
            routine();
        }
    }
    public void runAction(Action action) {
        if(opModeIsActive() && !isStopRequested()) {
            action.start();
        }
        while (!action.isFinished() && (opModeIsActive() && !isStopRequested())) {
            action.update();
            Robot.getInstance().outputToTelemetry(telemetry);
            telemetry.update();
        }

        if(opModeIsActive() && !isStopRequested()) {
            action.done();
        }
    }

    public void ThreadAction(final Action action){
        Runnable runnable = () -> runAction(action);

        if(opModeIsActive() && !isStopRequested())
            new Thread(runnable).start();
    }

    public void ParallelActions(List<Action> actions) {
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
                }
            }
        }
        for (Action action : actions) {
            action.done();
        }
    }
}
