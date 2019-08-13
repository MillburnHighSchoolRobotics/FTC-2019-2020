package org.firstinspires.ftc.teamcode.lib;



import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.threads.PositionMonitor;
import org.firstinspires.ftc.teamcode.threads.ThreadManager;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import static org.firstinspires.ftc.teamcode.lib.DriveConstants.encoderTicksToInches;


public class MohanBot extends DriveBase {
    private DcMotorEx leftFront, leftRear, rightRear, rightFront, ex1, ex2, ey;
    private List<DcMotorEx> motors;
    private List<DcMotorEx> encoders;

    public MohanBot(HardwareMap hardwareMap) {
        super();

        leftFront = hardwareMap.get(DcMotorEx.class, "lf");
        leftRear = hardwareMap.get(DcMotorEx.class, "lb");
        rightRear = hardwareMap.get(DcMotorEx.class, "rb");
        rightFront = hardwareMap.get(DcMotorEx.class, "rf");
        ex1 = (DcMotorEx) hardwareMap.dcMotor.get("lf");
        ex2 = (DcMotorEx) hardwareMap.dcMotor.get("lb");
        ey = (DcMotorEx) hardwareMap.dcMotor.get("rf");

        motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);
        encoders = Arrays.asList(ex1, ex2, ey);

        for (DcMotorEx motor : motors) {
            // TODO: decide whether or not to use the built-in velocity PID
            // if you keep it, then don't tune kStatic or kA
            // otherwise, comment out the following line
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);

        // TODO: set the tuned coefficients from DriveVelocityPIDTuner if using RUN_USING_ENCODER
        // setPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, ...);

        setLocalizer(new TrackingWheelLocalizer(hardwareMap));

        ThreadManager manager = ThreadManager.getInstance();
        manager.setHardwareMap(hardwareMap);
        manager.setupThread("PositionMonitor", PositionMonitor.class);
    }

    @Override
    public PIDCoefficients getPIDCoefficients(DcMotor.RunMode runMode) {
        PIDFCoefficients coefficients = leftFront.getPIDFCoefficients(runMode);
        return new PIDCoefficients(coefficients.p, coefficients.i, coefficients.d);
    }

    @Override
    public void setPIDCoefficients(DcMotor.RunMode runMode, PIDCoefficients coefficients) {
        for (DcMotorEx motor : motors) {
            motor.setPIDFCoefficients(runMode, new PIDFCoefficients(
                    coefficients.kP, coefficients.kI, coefficients.kD, 1
            ));
        }
    }

    @Override
    public List<Double> getWheelPositions() {
        List<Double> wheelPositions = new ArrayList<>();
        for (DcMotorEx motor : motors) {
            wheelPositions.add(encoderTicksToInches(motor.getCurrentPosition()));
        }
        return wheelPositions;
    }

    @Override
    public void setMotorPowers(double v, double v1, double v2, double v3) {
        leftFront.setPower(v);
        leftRear.setPower(v1);
        rightRear.setPower(v2);
        rightFront.setPower(v3);
    }

    @Override
    public double getRawExternalHeading() {
        return ThreadManager.getInstance().getValue("orientation", Double.class);
    }
}
