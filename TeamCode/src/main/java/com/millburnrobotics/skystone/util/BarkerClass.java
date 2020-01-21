package com.millburnrobotics.skystone.util;

import android.graphics.Bitmap;
import android.util.Log;

import com.millburnrobotics.skystone.Constants;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.opencv.android.OpenCVLoader;
import org.opencv.android.Utils;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

import java.util.LinkedList;
import java.util.List;

public class BarkerClass {
    static {
        if(OpenCVLoader.initDebug()) {
            Log.d("opencv","yay it works");
        } else {
            Log.d("opencv","nope it doesnt work");
        }
    }

    static private HardwareMap hardwareMap;
    static private VuforiaLocalizerImplSubclass vuforiaInstance;

    private static double BLUE_LINE_1_X = 100;
    private static double BLUE_LINE_2_X = 400;
    private static double BLUE_LINE_3_X = 800;
    private static double RED_LINE_1_X = 100;
    private static double RED_LINE_2_X = 400;
    private static double RED_LINE_3_X = 800;

    private int widthCamera;
    private int heightCamera;

    private VuforiaLocalizer.Parameters params;

    public BarkerClass(HardwareMap hardwareMap, Constants.SIDE side) {
        this.hardwareMap = hardwareMap;
        Constants.FieldConstants.side = side;
    }

    public void wake() {
        params = new VuforiaLocalizer.Parameters(hardwareMap.appContext.getResources().getIdentifier("Webcam1", "id", hardwareMap.appContext.getPackageName()));

        params.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        params.vuforiaLicenseKey = Constants.VUFORIA_KEY;

        vuforiaInstance = new VuforiaLocalizerImplSubclass(params);
    }

    public int bark() {
        widthCamera = vuforiaInstance.rgb.getBufferWidth();
        heightCamera = vuforiaInstance.rgb.getHeight();

        Mat img = new Mat();
        Bitmap bm = Bitmap.createBitmap(widthCamera, heightCamera, Bitmap.Config.RGB_565);
        bm.copyPixelsFromBuffer(vuforiaInstance.rgb.getPixels());
        Utils.bitmapToMat(bm, img);

        Mat blur = new Mat();
        Imgproc.GaussianBlur(img, blur, new Size(9,9),50);
        Mat gray = new Mat();
        Imgproc.cvtColor(blur, gray, Imgproc.COLOR_RGB2GRAY);

        Mat thresh = new Mat();
        Imgproc.threshold(gray,thresh, 60,255,Imgproc.THRESH_BINARY_INV);


        List<MatOfPoint> contours = new LinkedList<>();
        Imgproc.findContours(thresh, contours, new Mat(), Imgproc.RETR_CCOMP, Imgproc.CHAIN_APPROX_NONE);
        blur.release();
        thresh.release();

        double area = 0;
        int maxnum = -1;
        for (int c = 0; c < contours.size(); c++) {
            if (Imgproc.contourArea(contours.get(c)) > area) {
                area = Imgproc.contourArea(contours.get(c));
                maxnum = c;
            }
        }

        Point centroid = new Point();
        centroid.x = 0;
        centroid.y = 0;
        if (contours.size() > 0) {
            Moments moments = Imgproc.moments(contours.get(maxnum));
            if (moments.get_m00() != 0) {
                centroid.x = moments.get_m10() / moments.get_m00();
                centroid.y = moments.get_m01() / moments.get_m00();
            }
        }
        Log.d("watbark",""+centroid.x);
        Log.d("watbark","blue1 "+BLUE_LINE_1_X);
        Log.d("watbark","blue2 "+BLUE_LINE_2_X);
        Log.d("watbark","blue3 "+BLUE_LINE_3_X);
        Log.d("watbark","red1 "+RED_LINE_1_X);
        Log.d("watbark","red2 "+RED_LINE_2_X);
        Log.d("watbark","red3 "+RED_LINE_3_X);

        for (MatOfPoint mat : contours) {
            mat.release();
        }

        int pos = -1;
        if (Constants.FieldConstants.side == Constants.SIDE.BLUE) {
            if (centroid.x > BLUE_LINE_1_X && centroid.x < BLUE_LINE_2_X)
                pos = 1;
            else if (centroid.x > BLUE_LINE_2_X && centroid.x < BLUE_LINE_3_X)
                pos = 2;
            else if (centroid.x > BLUE_LINE_3_X)
                pos = 3;
        } else {
            if (centroid.x > RED_LINE_1_X && centroid.x < RED_LINE_2_X)
                pos = 1;
            else if (centroid.x > RED_LINE_3_X)
                pos = 2;
            else if (centroid.x > RED_LINE_2_X && centroid.x < RED_LINE_3_X)
                pos = 3;
        }

        Log.d("watbark",""+pos);
        if (pos == -1) {
            pos = 3;
        }
        return pos;
    }
}