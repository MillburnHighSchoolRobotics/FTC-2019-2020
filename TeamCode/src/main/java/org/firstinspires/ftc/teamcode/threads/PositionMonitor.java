package org.firstinspires.ftc.teamcode.threads;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robot.GlobalConstants;
import org.firstinspires.ftc.teamcode.util.MathUtils;

import static org.firstinspires.ftc.teamcode.robot.GlobalConstants.FPS_UPDATE_PERIOD;

public class PositionMonitor extends MonitorThread {
    private static final String TAG = "PositionMonitor";
    private DcMotorEx ex1;
    private DcMotorEx ex2;
    private DcMotorEx ey;

    double ex1PosLast = 0;
    double ex2PosLast = 0;
    double eyPosLast = 0;
    double offsetX = 15.5; // the left right distance from the x1 tracking wheel to the x2 tracking wheel
    double offsetY = -2.75; // the forward backward distance from the tracking center to the back tracking wheel

    double x = 0;
    double y = 0;
    double theta = 0;
    double orientation = 0;

    public PositionMonitor(Thread thread, HardwareMap hardwareMap, Pose2d start) {
        super(thread, hardwareMap, TAG);
        ex1 = (DcMotorEx) hardwareMap.dcMotor.get("lb");
        ex2 = (DcMotorEx) hardwareMap.dcMotor.get("lf");
        ey = (DcMotorEx) hardwareMap.dcMotor.get("rf");
        ex1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ex1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ex2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ex2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ey.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ey.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        x = start.getX();
        y = start.getY();
        theta = start.getHeading();

        setValue("x",start.getX());
        setValue("y",start.getY());
        setValue("theta",360-Math.toDegrees(start.getHeading()));
    }
    @Override
    protected void loop() {
        updatePosition();
        setValue("theta", Math.toDegrees(MathUtils.normalize(2*Math.PI-theta)));
        setValue("x", x);
        setValue("y", y);
    }
    protected void updatePosition() {
        double ex1Pos = GlobalConstants.encoderToDistance(ex1.getCurrentPosition());
        double ex2Pos = -GlobalConstants.encoderToDistance(ex2.getCurrentPosition());
        double eyPos = GlobalConstants.encoderToDistance(ey.getCurrentPosition());
        Log.d(TAG, "ex1 pos (inches): " + ex1Pos);
        Log.d(TAG, "ex2 pos (inches): " + ex2Pos);
        Log.d(TAG, "ey pos (inches): " + eyPos);

        double deltaEX1 = ex1Pos-ex1PosLast;
        double deltaEX2 = ex2Pos-ex2PosLast;
        double deltaEY = eyPos-eyPosLast;

        double dTheta = (deltaEX2-deltaEX1)/offsetX;
        double dX = (deltaEX1+deltaEX2)/2.0;
        double dY = deltaEY+offsetY*dTheta;

        Pose2d deltaEncoderPos = new Pose2d(dX,dY,dTheta);
        Vector2d deltaVector = ExponentialMap(deltaEncoderPos);
        Pose2d deltaPose = new Pose2d(deltaVector.rotated(theta),dTheta);

        Pose2d currentPose = new Pose2d(
                x+deltaPose.getX(),
                y+deltaPose.getY(),
                MathUtils.normalize(theta+deltaPose.getHeading())
        );
        x = currentPose.getX();
        y = currentPose.getY();
        theta = currentPose.getHeading();

        ex1PosLast = ex1Pos;
        ex2PosLast = ex2Pos;
        eyPosLast = eyPos;

        try {
            Thread.sleep(FPS_UPDATE_PERIOD);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }
    private Vector2d ExponentialMap(Pose2d deltaPos) {
        double dX = deltaPos.getX();
        double dY = deltaPos.getY();
        double dTheta = deltaPos.getHeading();

        double s,c;
        if (MathUtils.equals(Math.abs(dTheta),0,0.000001)) {
            s = 1 - dTheta * dTheta / 6.0;
            c = dTheta / 2.0;
        } else {
            s = Math.sin(dTheta)/dTheta;
            c = (1-Math.cos(dTheta))/dTheta;
        }

        return new Vector2d(dX*s-dY*c,dX*c+dY*s);
    }
}