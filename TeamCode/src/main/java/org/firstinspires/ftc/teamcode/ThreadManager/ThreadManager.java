package org.firstinspires.ftc.teamcode.ThreadManager;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;


import java.util.HashMap;
import java.util.Map;

public class ThreadManager {
    private HashMap<String, MonitorThread> monitorThreads;
    private HashMap<String, Object> values;
    private HashMap<String, Thread> owners;

    private HardwareMap hardwareMap;
    private LinearOpMode currentAuton;

    static final String TAG = "ThreadManager";
    public ThreadManager() {
        this.monitorThreads = new HashMap<>();
        this.values = new HashMap<>();
        this.owners = new HashMap<>();
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
        values.clear();
        owners.clear();
    }

    public synchronized void provision(String name, Class<? extends MonitorThread> threadClass, Object... args) {
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

    public synchronized void setValue(String name, Object value) {
        if (!values.containsKey(name)) {
            if (!monitorThreads.containsValue(Thread.currentThread())) {
                Log.e(TAG, "Failed to set " + name + " to " + Thread.currentThread() + " since it is not a registered MonitorThread");
            } else {
                values.put(name, value);
                owners.put(name, Thread.currentThread());
                Log.d(TAG, "Setting new thread as owner of " + name + ": " + Thread.currentThread());
            }
        } else if (owners.get(name) == Thread.currentThread()) {
            values.put(name, value);
        } else {
            Log.e(TAG, "Unable to set " + name + " since it is not owned by " + Thread.currentThread().getName());
        }
    }

    public synchronized Object getValue(String name) {
        return values.get(name);
    }

    public synchronized <T> T getValue(String name, Class<T> type) {
        Object obj = values.get(name);
        return obj == null ? null : type.cast(obj);
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