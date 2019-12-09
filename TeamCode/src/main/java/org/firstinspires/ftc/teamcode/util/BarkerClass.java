package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

import static org.firstinspires.ftc.teamcode.robot.GlobalConstants.*;

public class BarkerClass {
    static private HardwareMap hardwareMap;
    static private SkystoneDetector detector;

    public BarkerClass(HardwareMap hwmap) {
        hardwareMap = hwmap;
    }

    public void init() {
        side = SIDE.BLUE; //change according to side

        VuforiaLocalizer.Parameters params = new VuforiaLocalizer.Parameters(hardwareMap.appContext.getResources().getIdentifier("Webcam1", "id", hardwareMap.appContext.getPackageName()));
        params.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        params.vuforiaLicenseKey = VUFORIA_KEY;

        VuforiaLocalizerImplSubclass vuforiaInstance = new VuforiaLocalizerImplSubclass(params);

        detector = new SkystoneDetector(vuforiaInstance);
    }

    public int getPos() {
        int pos = detector.getPos();
        return pos;
    }
}