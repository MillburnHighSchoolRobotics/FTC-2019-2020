package org.firstinspires.ftc.teamcode.lib.MotionProfiling;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.lib.DriveBase;
import org.firstinspires.ftc.teamcode.lib.MohanBot;

/*
 * This is a simple routine to test turning capabilities.
 */
@Autonomous(group = "drive")
public class TurnTest extends LinearOpMode {
    public static double ANGLE = 90;

    @Override
    public void runOpMode() throws InterruptedException {
        DriveBase drive = new MohanBot(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        drive.turnSync(Math.toRadians(ANGLE));
    }
}
