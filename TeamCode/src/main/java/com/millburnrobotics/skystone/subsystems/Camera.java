package com.millburnrobotics.skystone.subsystems;

import android.graphics.Bitmap;
import android.util.Log;

import com.millburnrobotics.lib.util.VuforiaLocalizerImplSubclass;
import com.millburnrobotics.skystone.Constants;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.opencv.android.OpenCVLoader;
import org.opencv.android.Utils;
import org.opencv.core.Mat;
import org.opencv.core.MatOfByte;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

import java.io.FileOutputStream;
import java.io.IOException;
import java.util.Date;
import java.util.LinkedList;
import java.util.List;

public class Camera extends Subsystem {
    static {
        if(OpenCVLoader.initDebug()) {
            Log.d("opencv","yay it works");
        } else {
            Log.d("opencv","nope it doesnt work");
        }
    }
    private String TAG = "BarkerClass";
    private VuforiaLocalizerImplSubclass vuforiaInstance;

    private final static double BLUE_LINE_1_Y = 0;
    private final static double BLUE_LINE_2_Y = 215;
    private final static double BLUE_LINE_3_Y = 550;
    private final static double RED_LINE_1_Y = 100/2.7;
    private final static double RED_LINE_2_Y = 650/2.7;
    private final static double RED_LINE_3_Y = 1250/2.7;
    private final static double RED_LINE_4_Y = 1850/2.7;
    private final static double croppingConstant = 0.4;

    @Override
    public void init(boolean auto) {
        if (auto) {
            VuforiaLocalizer.Parameters params = new VuforiaLocalizer.Parameters(Robot.getInstance().getCameraMonitorViewerID());
            params.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
            params.vuforiaLicenseKey = Constants.VUFORIA_KEY;
            vuforiaInstance = new VuforiaLocalizerImplSubclass(params);
        }
    }

    @Override
    public void outputToTelemetry(Telemetry telemetry) {
        telemetry.addData(TAG, Robot.getInstance().block);
    }

    @Override
    public void update() {

    }
    public void detectBlock() {
        int heightCamera = vuforiaInstance.rgb.getHeight();
        int widthCamera = vuforiaInstance.rgb.getWidth();

        Mat img = new Mat();
        Bitmap bm = Bitmap.createBitmap(widthCamera, heightCamera, Bitmap.Config.RGB_565);
        bm.copyPixelsFromBuffer(vuforiaInstance.rgb.getPixels());
        Utils.bitmapToMat(bm, img);

        Mat rgb = new Mat();
        Imgproc.cvtColor(img, rgb, Imgproc.COLOR_BGR2RGB);
        img.release();

        int width1 = (int)Math.round(rgb.size().width);
        int width = (int)(croppingConstant*width1);
        int height = (int)Math.round(rgb.size().height);

        Log.d(TAG, rgb.size().height + " " + rgb.size().width);

        Mat crop = rgb.clone().submat(0,height,0,width);

        Log.d(TAG, crop.size().height + " " + crop.size().width);

        Mat blur = new Mat();
        Imgproc.GaussianBlur(crop, blur, new Size(9,9),50);
        Mat gray = new Mat();
        Imgproc.cvtColor(blur, gray, Imgproc.COLOR_RGB2GRAY);

        Mat thresh = new Mat();
        Imgproc.threshold(gray,thresh, 10,255,Imgproc.THRESH_BINARY_INV);


        List<MatOfPoint> contours = new LinkedList<>();
        Imgproc.findContours(thresh, contours, new Mat(), Imgproc.RETR_CCOMP, Imgproc.CHAIN_APPROX_NONE);
        Imgproc.drawContours(crop,contours,-1,new Scalar(0,255,0),3);

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
                Imgproc.circle(crop, new Point((int)centroid.x,(int)centroid.y),20, new Scalar(255),2);
            }
        }
        Log.d(TAG, "(" + centroid.x + "," + centroid.y + ")");

        if (Robot.getInstance().side == Constants.Side.BLUE) {
            Imgproc.line(crop,new Point(0,BLUE_LINE_1_Y),new Point(height,BLUE_LINE_1_Y),new Scalar(255,0,0),5);
            Imgproc.line(crop,new Point(0,BLUE_LINE_2_Y),new Point(height,BLUE_LINE_2_Y),new Scalar(255,0,0),5);
            Imgproc.line(crop,new Point(0,BLUE_LINE_3_Y),new Point(height,BLUE_LINE_3_Y),new Scalar(255,0,0),5);
        } else {
            Imgproc.line(crop,new Point(0,RED_LINE_1_Y),new Point(height,RED_LINE_1_Y),new Scalar(255,0,0),5);
            Imgproc.line(crop,new Point(0,RED_LINE_2_Y),new Point(height,RED_LINE_2_Y),new Scalar(255,0,0),5);
            Imgproc.line(crop,new Point(0,RED_LINE_3_Y),new Point(height,RED_LINE_3_Y),new Scalar(255,0,0),5);
            Imgproc.line(crop,new Point(0,RED_LINE_4_Y),new Point(height,RED_LINE_4_Y),new Scalar(255,0,0),5);
        }

        try {
            recordImg(rgb, "Camera Image");
            recordImg(crop, "Cropped Image");
            recordImg(thresh, "Thresh Image");
        } catch (IOException e) {
            Log.d(TAG, "Error Writing Image");
            e.printStackTrace();
        }

        for (MatOfPoint mat : contours) {
            mat.release();
        }

        if (Robot.getInstance().side == Constants.Side.BLUE) {
            if (centroid.y > BLUE_LINE_1_Y && centroid.y < BLUE_LINE_2_Y)
                Robot.getInstance().block = Constants.Block.LEFT;
            else if (centroid.y > BLUE_LINE_2_Y && centroid.y < BLUE_LINE_3_Y)
                Robot.getInstance().block = Constants.Block.CENTER;
            else if (centroid.y > BLUE_LINE_3_Y) {
                Robot.getInstance().block = Constants.Block.RIGHT;
            }
        } else {
            if (centroid.y > RED_LINE_1_Y && centroid.y < RED_LINE_2_Y || centroid.y > RED_LINE_4_Y)
                Robot.getInstance().block = Constants.Block.RIGHT;
            else if (centroid.y > RED_LINE_3_Y)
                Robot.getInstance().block = Constants.Block.CENTER;
            else if (centroid.y > RED_LINE_2_Y && centroid.y < RED_LINE_3_Y) {
                Robot.getInstance().block = Constants.Block.LEFT;
            }
        }
    }
    public static void recordImg(Mat mat, String name) throws IOException {
        final MatOfByte buf = new MatOfByte();
        Imgcodecs.imencode(".jpeg", mat, buf);
        String directory = "/sdcard/8405Logging/";
        FileOutputStream out = new FileOutputStream(directory+new Date(System.currentTimeMillis()).toString()+name+".jpeg");
        out.write(buf.toArray());
        out.close();
    }
}
