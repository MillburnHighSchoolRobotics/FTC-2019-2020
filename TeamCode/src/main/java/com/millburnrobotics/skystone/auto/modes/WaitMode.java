package com.millburnrobotics.skystone.auto.modes;

import com.millburnrobotics.skystone.auto.AutoModeBase;
import com.millburnrobotics.skystone.subsystems.Robot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(group = "Test")
public class WaitMode extends AutoModeBase {

    @Override
    public void routine() {
        Robot.getInstance().lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        Robot.getInstance().lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        Robot.getInstance().rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        Robot.getInstance().rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        while(!isStopRequested() && opModeIsActive());
    }
}
