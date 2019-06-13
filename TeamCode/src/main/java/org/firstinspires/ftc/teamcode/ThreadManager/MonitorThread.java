package org.firstinspires.ftc.teamcode.ThreadManager;

import android.util.Log;

import com.qualcomm.robotcore.hardware.HardwareMap;

public abstract class MonitorThread extends Thread {
    protected final Thread thread;
    protected final HardwareMap hardwareMap;
    public MonitorThread(Thread thread, HardwareMap hardwareMap) {
        this.thread = thread;
        this.hardwareMap = hardwareMap;
    }
    @Override
    public void run() {
        while (!Thread.currentThread().isInterrupted() && !thread.isInterrupted()) {
            if (!(ThreadManager.getInstance().getCurrentAuton() == null || (ThreadManager.getInstance().getCurrentAuton().opModeIsActive() || !ThreadManager.getInstance().getCurrentAuton().isStarted()))) {
                ThreadManager.getInstance().clean();
                break;
            }
            loop();
            try {
                Thread.sleep(10);
            } catch (InterruptedException e) {
                Log.e(ThreadManager.TAG, e.getMessage(), e);
                break;
            }
        }
    }
    protected abstract void loop();
}