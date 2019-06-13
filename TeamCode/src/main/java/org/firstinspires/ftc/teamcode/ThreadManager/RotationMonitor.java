package org.firstinspires.ftc.teamcode.ThreadManager;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class RotationMonitor extends MonitorThread {
    private DcMotorEx ex1;
    private DcMotorEx ex2;
    public RotationMonitor(Thread thread, HardwareMap hardwareMap) {
        super(thread, hardwareMap, "RotationMonitor");
        ex1 = (DcMotorEx) hardwareMap.dcMotor.get("lf");
        ex2 = (DcMotorEx) hardwareMap.dcMotor.get("lb");
        ex1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ex1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ex2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ex2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    @Override
    protected void loop() {
        setValue("rotation", 0);
    }
}