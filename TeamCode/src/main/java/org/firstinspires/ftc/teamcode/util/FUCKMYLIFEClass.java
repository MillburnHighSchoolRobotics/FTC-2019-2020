package org.firstinspires.ftc.teamcode.util;

import android.graphics.Bitmap;
import android.util.Log;

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

public class FUCKMYLIFEClass {
    static {
        OpenCVLoader.initDebug();
    }
    VuforiaLocalizerImplSubclass vuforiaInstance;

    private int widthCamera;
    private int heightCamera;
    public FUCKMYLIFEClass(VuforiaLocalizerImplSubclass vuforiaInstance) {
        this.vuforiaInstance = vuforiaInstance;
        widthCamera = vuforiaInstance.rgb.getBufferWidth();
        heightCamera = vuforiaInstance.rgb.getHeight();
    }

    public int getPos() {
        Mat img = new Mat();
        Bitmap bm = Bitmap.createBitmap(widthCamera, heightCamera, Bitmap.Config.RGB_565);
        bm.copyPixelsFromBuffer(vuforiaInstance.rgb.getPixels());
        Utils.bitmapToMat(bm, img);

        Mat blur = new Mat();
        Imgproc.GaussianBlur(img, blur, new Size(9,9),50);
        Mat gray = new Mat();
        Imgproc.cvtColor(blur, gray, Imgproc.COLOR_RGB2GRAY);

        Mat thresh = new Mat();
        Imgproc.threshold(gray,thresh,65,255,Imgproc.THRESH_BINARY_INV);


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
        for (MatOfPoint mat : contours) {
            mat.release();
        }

        int pos = -1;
        if ((centroid.x >= 0) && (centroid.x < (img.cols()/3.0))) {
            pos = 1;
        } else if ((centroid.x >= (img.cols()/3)) && (centroid.x < (2*(img.cols()/3)))) {
            pos = 2;
        } else if (centroid.x >= (2*(img.cols()/3))) {
            pos = 3;
        }
        Log.d("detection",""+pos);
        if (pos == -1) {
            pos = 3;
        }
        return pos;
    }
}