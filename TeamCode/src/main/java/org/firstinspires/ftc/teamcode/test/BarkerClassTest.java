package org.firstinspires.ftc.teamcode.test;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.MohanBot;
import org.firstinspires.ftc.teamcode.util.BarkerClass;


@Autonomous(group = "auton")
@Disabled
public class BarkerClassTest extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        MohanBot drive = new MohanBot(hardwareMap,this);
        BarkerClass barker = new BarkerClass(hardwareMap);

        barker.setupCam();

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectory(
                drive.trajectoryBuilder()
                        .forward(10)
                        .build()
        );

        int pos = barker.getPos();
        telemetry.addData("barker",pos);
        telemetry.update();
        Log.d("barker",pos+"");
        while (!isStopRequested()) {
            Thread.sleep(10);
        }
    }
}
