package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.graphics.Bitmap;
import android.os.Environment;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.CvException;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.Date;

import static org.opencv.imgproc.Imgproc.COLOR_RGB2HSV;
import static org.opencv.imgproc.Imgproc.CV_HOUGH_GRADIENT;

@Autonomous(name = "OpenCV Beacon", group = "Test Code")
public class OpenCVBeacon extends LinearOpMode implements CameraBridgeViewBase.CvCameraViewListener2 {
    private CameraBridgeViewBase mOpenCvCameraView;
    private Mat rgb;
    private Mat hsv;
    private Mat gray;

    public void runOpMode() {
        BaseLoaderCallback mLoaderCallback = new BaseLoaderCallback(hardwareMap.appContext) {
            @Override
            public void onManagerConnected(int status) {
                switch (status) {
                    case LoaderCallbackInterface.SUCCESS: {
                        mOpenCvCameraView.enableView();
                    }
                    break;
                    default: {
                        super.onManagerConnected(status);
                    }
                    break;
                }
            }
        };

        mOpenCvCameraView = (CameraBridgeViewBase) ((Activity) hardwareMap.appContext).findViewById(R.id.surfaceView);
        mOpenCvCameraView.setVisibility(CameraBridgeViewBase.VISIBLE);
        mOpenCvCameraView.setCameraIndex(CameraBridgeViewBase.CAMERA_ID_FRONT);
        mOpenCvCameraView.setCvCameraViewListener(this);

        if (!OpenCVLoader.initDebug()) {
            OpenCVLoader.initAsync(OpenCVLoader.OPENCV_VERSION_3_0_0, hardwareMap.appContext, mLoaderCallback);
        } else {
            mLoaderCallback.onManagerConnected(LoaderCallbackInterface.SUCCESS);
        }

        waitForStart();
        while(opModeIsActive()) {
            idle();
        }

        if (mOpenCvCameraView != null) {
            mOpenCvCameraView.disableView();
        }
    }

    public void onCameraViewStarted(int width, int height) {
        rgb = new Mat(height, width, CvType.CV_8UC4);
        hsv = new Mat(height, width, CvType.CV_8UC4);
        gray = new Mat(height, width, CvType.CV_8UC1);
    }

    public void onCameraViewStopped() {
        rgb.release();
        hsv.release();
        gray.release();
    }

    public Mat onCameraFrame(CameraBridgeViewBase.CvCameraViewFrame inputFrame) {
        rgb.release();
        hsv.release();
        gray.release();

        rgb = inputFrame.rgba();
        gray = inputFrame.gray();

        Mat circles = new Mat();

        Imgproc.cvtColor(rgb, hsv, COLOR_RGB2HSV);
        Imgproc.blur(hsv, hsv, new Size(9, 9));

        String color = "red";
        if(color.equals("red")) {
            Mat lowerRed = new Mat();
            Core.inRange(hsv, new Scalar(0, 230, 65), new Scalar(13, 255, 145), lowerRed); //345, 84, 41 /// 359, 100, 74

            Mat upperRed = new Mat();
            Core.inRange(hsv, new Scalar(145, 20, 125), new Scalar(179, 165, 255), upperRed); //324, 19, 62 /// 55, 62, 100

            Core.bitwise_or(lowerRed, upperRed, gray); //Bitwise or combines the ranges

            Core.bitwise_not(gray, gray); //Reverse colors, correct button is white surrounded by black

            Imgproc.HoughCircles(gray, circles, CV_HOUGH_GRADIENT, 2, 300, 200, 60, 0, 100);
        } else {
            Scalar min = new Scalar(65, 45, 140); //180, 29, 67
            Scalar max = new Scalar(140, 255, 255); //238, 100, 100
            Core.inRange(hsv, min, max, gray); //Correct button is black surrounded by white
            Core.bitwise_not(gray, gray); //Reverse colors, correct button is white surrounded by black

            Imgproc.HoughCircles(gray, circles, CV_HOUGH_GRADIENT, 2, 250, 200, 60, 0, 200);
        }

        Log.i("CIRCLE", "Detected: " + String.valueOf(circles.cols()));
        for(int i = 0; i < circles.cols(); i++) {
            double[] circleVector = circles.get(0, i);

            Point center = new Point(Math.round(circleVector[0]), Math.round(circleVector[1]));
            int radius = (int) Math.round(circleVector[2]);

            Imgproc.circle(gray, center, 3, new Scalar(0, 255, 0), -1);
            Imgproc.circle(gray, center, radius, new Scalar(0, 0, 255), 10);

        }

        return gray;
    }
}