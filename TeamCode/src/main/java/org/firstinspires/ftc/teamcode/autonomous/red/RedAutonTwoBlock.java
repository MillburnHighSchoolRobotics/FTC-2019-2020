package org.firstinspires.ftc.teamcode.autonomous.red;

import android.service.quicksettings.Tile;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.heading.SplineInterpolator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.robot.MohanBot;
import org.firstinspires.ftc.teamcode.util.BarkerClass;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
import static org.firstinspires.ftc.teamcode.robot.GlobalConstants.*;


@Autonomous(group = "auton")
public class RedAutonTwoBlock extends LinearOpMode {

    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private static final boolean PHONE_IS_PORTRAIT = false  ;

    private static final String VUFORIA_KEY =
            "AcSW/tj/////AAABmUB3byzZQksfqhymb0Tr3M92yvtrzF4HgDl0t7Z07OZ2xscXR1yyeX4GxftrShvm9T926ZCW0VglQKXuQE5+JkrABVijohk5DCkcE9CcxHy3mTs2Ui76Nz+9CQTgOsr6/AMLV+Te6uyXTs3rZwGdnHGRo0Q1yboJCQ51Ap2rgJc//ehVdkp/QetIMnfhRffac0edAHFt0i2F5++S/OH/4kdxFd5ha0lswd4nTnqU2MiJrz+OH4WQPQ8JC94dQZ6F3m/iX5mk4TCq/9xg3cTJvFccEUawf7PIsapABxKMJB6hcPikwa0XtyGB+vEb7fQAXZ80tRal2mcwKSHrDM4ZvYisD73X+sTIAqQnXgxYiL14";
    private static final float stoneZ = 2.00f * 25.4f;

    private OpenGLMatrix lastLocation = null;
    private VuforiaLocalizer vuforia = null;
    WebcamName webcamName = null;

    private boolean targetVisible = false;
    private float phoneXRotate    = 0;
    private float phoneYRotate    = 0;
    private float phoneZRotate    = 0;

    private static double BOT_WIDTH = 18.0;
    private static double DEFAULT_STRAFE_POWER = 0.6;
    private static double DEFAULT_ROTATION_POWER = 0.4;
    private static double BASEPLATE_ALIGNMENT_Y = 49; //24 + 8 + 34/2

    @Override
    public void runOpMode() throws InterruptedException {
        //forwards, to wall, on first tile beyond bridge, sideplate on tile edge nearest to audience
//        BarkerClass barker = new BarkerClass(hardwareMap, SIDE.RED);

        MohanBot robot = new MohanBot(hardwareMap, this, new Pose2d(63, -2*TILE_WIDTH + BOT_WIDTH/2, Math.toRadians(90)));

        robot.intake.intakeStop();

        waitForStart();

        if (isStopRequested()) return;

//        int pos = barker.bark();
        int pos = 2;

        if (pos == 1) {
            robot.moveTo(new Vector2d(1.5*TILE_WIDTH, -7.0/4.0*TILE_WIDTH+4), 50,DEFAULT_STRAFE_POWER,DEFAULT_ROTATION_POWER);
            robot.intake.intakeIn();
            robot.strafeTo(new Vector2d(18, -1*TILE_WIDTH),DEFAULT_STRAFE_POWER);
        }
        else if (pos == 2) {
            robot.moveTo(new Vector2d(1.5*TILE_WIDTH, -7.0/4.0*TILE_WIDTH), 50, DEFAULT_STRAFE_POWER,DEFAULT_ROTATION_POWER);
            robot.intake.intakeIn();
            robot.strafeTo(new Vector2d(18, -1*TILE_WIDTH-4),DEFAULT_STRAFE_POWER);
        } else {
            robot.moveTo(new Vector2d(24, -24), 180,DEFAULT_STRAFE_POWER,DEFAULT_ROTATION_POWER);
            robot.intake.intakeIn();
            robot.strafeTo(new Vector2d(24, -32),DEFAULT_STRAFE_POWER);
        }
        robot.intake.intakeStop();
        robot.moveTo(new Vector2d(48, -24), 0,0.6,0.4);
        robot.moveTo(new Vector2d(48, 0), 0,0.6,0.4);

        robot.intake.intakeOut();
        Thread.sleep(1000);



        //second block
//            if (pos == 1) {
//
//            }
//            if (pos == 2) {
//
//            }
//            else {
//                robot.moveTo(new Vector2d(30, -44), 180,DEFAULT_STRAFE_POWER,0.4);
//                robot.moveTo(new Vector2d(24, -44), 180,DEFAULT_STRAFE_POWER,0.4);
//                robot.intake.intakeIn();
//                robot.strafeTo(new Vector2d(20, -52),DEFAULT_STRAFE_POWER);
//            }
//
//        robot.intake.intakeStop();
//        robot.moveTo(new Vector2d(48, -24), 0,0.6,0.4);
//        robot.moveTo(new Vector2d(48, 0), 0,0.6,0.4);
//
//        robot.intake.intakeOut();
//        Thread.sleep(1000);
    }
}
