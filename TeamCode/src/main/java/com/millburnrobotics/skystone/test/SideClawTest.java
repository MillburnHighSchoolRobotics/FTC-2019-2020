//package com.millburnrobotics.skystone.test;
//
//import android.util.Log;
//
//import com.millburnrobotics.lib.util.MathUtils;
//import com.millburnrobotics.skystone.Constants;
//import com.millburnrobotics.skystone.Constants.*;
//import com.millburnrobotics.skystone.subsystems.Subsystem;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.Servo;
//
//import static com.millburnrobotics.skystone.Constants.SideClawConstants._SideClawArmL;
//import static com.millburnrobotics.skystone.Constants.SideClawConstants._SideClawArmR;
//import static com.millburnrobotics.skystone.Constants.SideClawConstants._SideClawClawL;
//import static com.millburnrobotics.skystone.Constants.SideClawConstants._SideClawClawR;
//
//
//@TeleOp(group = "util")
//public class SideClawTest extends OpMode {
//    SubsystemTestSide side;
//
//    public SideClawTest(SubsystemTestSide side) {
//        this.side = side;
//    }
//
//    double barPos, clampPos, inc;
//    Servo sideClawBar, sideClawClamp;
//
//    @Override
//    public void init() {
//        if (side == SubsystemTestSide.LEFT) {
//            sideClawBar = hardwareMap.servo.get(_SideClawArmL);
//            sideClawClamp = hardwareMap.servo.get(_SideClawClawL);
//        } else if (side == SubsystemTestSide.RIGHT) {
//            sideClawBar = hardwareMap.servo.get(_SideClawArmR);
//            sideClawClamp = hardwareMap.servo.get(_SideClawClawR);
//        }
//        barPos = clampPos = 0.5;
//        inc = 0.1;
//    }
//
//    @Override
//    public void loop() {
//        try {
//            Thread.sleep(50);
//        } catch (InterruptedException ie) {
//            Log.d("IE", ie+"");
//        }
//
//        if ((gamepad1.dpad_up || gamepad1.left_bumper) && clampPos < 1) {
//            clampPos += inc;
//        }
//        if ((gamepad1.dpad_down || MathUtils.equals(gamepad1.left_trigger, 1, 0.05)) && clampPos > 0) {
//            clampPos -= inc;
//        }
//
//        if ((gamepad1.dpad_left || gamepad1.right_bumper) && barPos < 1) {
//            barPos += inc;
//        }
//        if ((gamepad1.dpad_right || MathUtils.equals(gamepad1.right_trigger, 1, 0.05)) && barPos > 0) {
//            barPos -= inc;
//        }
//
//        sideClawBar.setPosition(barPos);
//        sideClawClamp.setPosition(clampPos);
//
//        telemetry.addData("Bar",barPos+"");
//        telemetry.addData("Clamp",clampPos+"");
//        telemetry.addData("Inc",inc+"");
//        telemetry.update();
//
//        Log.d("Bar",barPos+"");
//        Log.d("Clamp",clampPos+"");
//        Log.d("Inc",inc+"");
//    }
//
//}
