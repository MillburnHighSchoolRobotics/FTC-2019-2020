package com.millburnrobotics.skystone.test;

import com.millburnrobotics.lib.geometry.Pose;
import com.millburnrobotics.skystone.auto.AutoModeBase;
import com.millburnrobotics.skystone.auto.actions.WaitAction;
import com.millburnrobotics.skystone.subsystems.Robot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(group = "Test")
public class WaitMode extends AutoModeBase {

    @Override
    public void routine() {
        Robot.getInstance().getOdometry().setPose(new Pose(0,0,0));
        Robot.getInstance().lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        Robot.getInstance().lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        Robot.getInstance().rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        Robot.getInstance().rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        while(!isStopRequested() && opModeIsActive());
    }
}
