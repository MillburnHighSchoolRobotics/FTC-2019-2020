package com.millburnrobotics.skystone.auto.modes;

import com.millburnrobotics.skystone.auto.actions.Action;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

public abstract class AutoModeBase extends LinearOpMode {

    protected ElapsedTime autoTimer = new ElapsedTime();

    public abstract void routine();

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        autoTimer.reset();

//        ThreadAction(new RobotUpdate(autoTimer));

        if (opModeIsActive() && !isStopRequested()) {
            routine();
        }

//        done();
    }
    public void runAction(Action action) {
        if(opModeIsActive() && !isStopRequested()) {
            action.start();
        }
        while (!action.isFinished() && (opModeIsActive() && !isStopRequested())) {
            action.update();
//            robot.outputToTelemetry(telemetry);
//            telemetry.update();
        }

        if(opModeIsActive() && !isStopRequested()) {
            action.done();
        }
    }

//    public void ThreadAction(final Action action){
//        Runnable runnable = new Runnable() {
//            @Override
//            public void run() {
//                runAction(action);
//            }
//        };
//
//        if(opModeIsActive() && !isStopRequested())
//            new Thread(runnable).start();
//    }
}
