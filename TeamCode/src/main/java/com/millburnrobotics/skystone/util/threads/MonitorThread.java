package com.millburnrobotics.skystone.util.threads;

import android.util.Log;

import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.HashMap;

public abstract class MonitorThread extends Thread {
    protected final Thread thread;
    protected final HardwareMap hardwareMap;
    protected volatile HashMap values;
    private String TAG;
    public MonitorThread(Thread thread, HardwareMap hardwareMap, String TAG) {
        this.thread = thread;
        this.hardwareMap = hardwareMap;
        this.TAG = TAG;
        values = new HashMap<>();
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

    protected synchronized void setValue(String key, Object value) {
        values.put(key,value);
        Log.d(TAG, "Setting value of " + key + " to " + value.toString());
    }
    protected synchronized HashMap getValues() {
        return values;
    }
}