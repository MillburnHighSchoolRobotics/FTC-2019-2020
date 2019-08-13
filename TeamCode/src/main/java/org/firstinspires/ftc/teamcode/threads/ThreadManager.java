package org.firstinspires.ftc.teamcode.threads;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;


import java.util.HashMap;
import java.util.Map;

public class ThreadManager {
    private HashMap<String, MonitorThread> monitorThreads;

    private HardwareMap hardwareMap;
    private LinearOpMode currentAuton;

    static final String TAG = "ThreadManager";
    public ThreadManager() {
        this.monitorThreads = new HashMap<>();
        this.hardwareMap = null;
        this.currentAuton = null;
    }

    public synchronized void clean() {
        this.currentAuton = null;
        for (Map.Entry<String, MonitorThread> monitorThread : monitorThreads.entrySet()) {
            Log.d(TAG, "Killing Thread " + monitorThread.getKey());
            if (monitorThread.getValue().isAlive()) {
                monitorThread.getValue().interrupt();
                Log.d(TAG, "Killed Thread " + monitorThread.getKey());
            }
        }
        monitorThreads.clear();
    }

    public synchronized void setupThread(String name, Class<? extends MonitorThread> threadClass, Object... args) {
        Object[] passedArgs = new Object[args.length + 2];
        passedArgs[0] = Thread.currentThread();
        passedArgs[1] = hardwareMap;
        System.arraycopy(args, 0, passedArgs, 2, args.length);
        Class argTypes[] = new Class[passedArgs.length];
        for (int i = 0; i < passedArgs.length; i++) {
            argTypes[i] = passedArgs[i].getClass();
        }
        MonitorThread t;
        try {
            t = threadClass.getDeclaredConstructor(argTypes).newInstance(passedArgs);
        } catch (Exception e) {
            Log.e(TAG, e.getMessage(), e);
            throw new RuntimeException();
        }
        monitorThreads.put(name, t);
        t.start();
    }

    public synchronized Object getValue(String name) {
        Object val = null;
        for (Map.Entry<String, MonitorThread> monitorThread : monitorThreads.entrySet()) {
            if (monitorThread.getValue().getValues().containsKey(name)) {
                val = monitorThread.getValue().getValues().get(name);
            }
        }
        return val;
    }

    public synchronized <T> T getValue(String name, Class<T> type) {
        Object val = this.getValue(name);
        return val == null ? null : type.cast(val);
    }

    public synchronized void remove(String name) {
        MonitorThread t = monitorThreads.get(name);
        t.interrupt();
    }

    private static final ThreadManager INSTANCE = new ThreadManager();

    public static synchronized ThreadManager getInstance() {
        return INSTANCE;
    }

    public void setHardwareMap(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    public LinearOpMode getCurrentAuton() {
        return currentAuton;
    }

    public void setCurrentAuton(LinearOpMode currentAuton) {
        this.currentAuton = currentAuton;
    }
}