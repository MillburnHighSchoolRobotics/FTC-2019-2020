package com.millburnrobotics.skystone.auto;

import com.millburnrobotics.skystone.Constants;
import com.millburnrobotics.skystone.auto.actions.Action;
import com.millburnrobotics.skystone.auto.actions.RobotUpdateAction;
import com.millburnrobotics.skystone.subsystems.Robot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;

import static com.millburnrobotics.skystone.Constants.UPDATE_PERIOD;

public abstract class AutoModeBase extends LinearOpMode {

    protected ElapsedTime autoTimer = new ElapsedTime();
    private Constants.Side side;

    public abstract void routine();

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.setMsTransmissionInterval(50);
        Robot.getInstance().init(hardwareMap, true);
        waitForStart();

        autoTimer.reset();
        threadAction(new RobotUpdateAction(autoTimer));

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

    public void threadAction(final Action action){
        Runnable runnable = () -> runAction(action);

        if(opModeIsActive() && !isStopRequested())
            new Thread(runnable).start();
    }

    public void parallelActions(List<Action> actions) {
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
}
