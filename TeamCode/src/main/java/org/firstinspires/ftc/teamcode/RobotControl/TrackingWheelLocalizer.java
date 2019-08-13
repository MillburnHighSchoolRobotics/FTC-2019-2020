package org.firstinspires.ftc.teamcode.RobotControl;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Arrays;
import java.util.List;

/*
 * Sample tracking wheel localizer implementation assuming the standard configuration:
 *
 *    /--------------\
 *    |     ____     |
 *    |     ----     |
 *    | ||        || |
 *    | ||        || |
 *    |              |
 *    |              |
 *    \--------------/
 *
 * Note: this could be optimized significantly with REV bulk reads
 */

public class TrackingWheelLocalizer extends ThreeTrackingWheelLocalizer {
    public static double TICKS_PER_REV = 360*4;
    public static double WHEEL_RADIUS = (60/25.4)/2; // in
    public static double GEAR_RATIO = 32/48.0; // output (wheel) speed / input (encoder) speed

    public static double LATERAL_DISTANCE = 15.53; // in; distance between the left and right wheels
    public static double FORWARD_OFFSET = 5; // in; offset of the lateral wheel

    private DcMotor leftEncoder, rightEncoder, frontEncoder;

    public TrackingWheelLocalizer(HardwareMap hardwareMap) {
        super(Arrays.asList(
                new Vector2d(0, LATERAL_DISTANCE / 2), // left
                new Vector2d(0, -LATERAL_DISTANCE / 2), // right
                new Vector2d(FORWARD_OFFSET, 0) // front
        ), Arrays.asList(0.0, 0.0, Math.toRadians(90.0)));

        leftEncoder = hardwareMap.dcMotor.get("lf");
        rightEncoder = hardwareMap.dcMotor.get("rb");
        frontEncoder = hardwareMap.dcMotor.get("rf");
    }

    public static double encoderTicksToInches(int ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    @Override
    public List<Double> getWheelPositions() {
        return Arrays.asList(
                encoderTicksToInches(leftEncoder.getCurrentPosition()),
                encoderTicksToInches(rightEncoder.getCurrentPosition()),
                encoderTicksToInches(frontEncoder.getCurrentPosition())
        );
    }
}
