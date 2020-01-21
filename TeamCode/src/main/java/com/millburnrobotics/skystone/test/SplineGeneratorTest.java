package com.millburnrobotics.skystone.test;

import android.util.Log;

import com.millburnrobotics.lib.control.Path;
import com.millburnrobotics.lib.control.PathGenerator;
import com.millburnrobotics.lib.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


@Autonomous(group = "test")
public class SplineGeneratorTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        if (isStopRequested()) return;

        PathGenerator generator = new PathGenerator(new Pose(0,0,0));
        Path path = generator.strafeTo(new Pose(24,36)).path;
        Log.d("SplineGeneratorTest", "Start: " + path.start());
        Log.d("SplineGeneratorTest", "End: " + path.end());
        Log.d("SplineGeneratorTest", "Length: " + path.length());
        Log.d("SplineGeneratorTest", "Get(10): " + path.get(10));
    }
}
