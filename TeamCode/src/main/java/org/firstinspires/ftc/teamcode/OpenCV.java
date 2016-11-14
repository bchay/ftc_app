package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfInt;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

import static org.opencv.imgproc.Imgproc.COLOR_BGR2GRAY;
import static org.opencv.imgproc.Imgproc.COLOR_BGR2HSV;
import static org.opencv.imgproc.Imgproc.MORPH_RECT;
import static org.opencv.imgproc.Imgproc.boundingRect;
import static org.opencv.imgproc.Imgproc.contourArea;

@Autonomous(name = "OpenCV", group = "Test Code")
public class OpenCV extends LinearOpMode implements CameraBridgeViewBase.CvCameraViewListener2 {
    private CameraBridgeViewBase mOpenCvCameraView;
    private Mat mat;
    private Mat lower;
    private Mat upper;
    private Mat rgb;
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
        mOpenCvCameraView.setCameraIndex(CameraBridgeViewBase.CAMERA_ID_BACK);
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
        mat = new Mat(height, width, CvType.CV_8UC4);
        lower = new Mat(height, width, CvType.CV_8UC4);
        upper = new Mat(height, width, CvType.CV_8UC4);
        rgb = new Mat(height, width, CvType.CV_8UC4);
        gray = new Mat(height, width, CvType.CV_8UC1);
    }

    public void onCameraViewStopped() {
        mat.release();
        lower.release();
        upper.release();
        rgb.release();
        gray.release();
    }

    public Mat onCameraFrame(CameraBridgeViewBase.CvCameraViewFrame inputFrame) {
        mat.release();
        rgb.release();
        gray.release();
        lower.release();
        upper.release();

        rgb = inputFrame.rgba();

        Imgproc.cvtColor(rgb, mat, COLOR_BGR2HSV);
        Imgproc.cvtColor(rgb, gray, COLOR_BGR2GRAY);

        //Blue Code - TODO
        //Scalar min = new Scalar(100, 50, 50); //
        //Scalar max = new Scalar(140, 255, 255);

        //Red Code - Black box
        Scalar min = new Scalar(220/2, 190, 80);
        Scalar max = new Scalar(260, 255, 255);

        Core.inRange(mat, min, max, mat);

        Mat str_el = Imgproc.getStructuringElement(MORPH_RECT, new Size(3, 3));
        Imgproc.morphologyEx(mat, mat, Imgproc.MORPH_OPEN, str_el);
        Imgproc.morphologyEx(mat, mat, Imgproc.MORPH_CLOSE, str_el);

        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();

        Imgproc.findContours(mat, contours, hierarchy, Imgproc.RETR_CCOMP, Imgproc.CHAIN_APPROX_SIMPLE);

        int largest_contour_index = 0;
        Rect bounding_rect = null;
        double largest_area = 0;

        // iterate through each contour.
        for(int i = 0; i < contours.size(); i++ ) {
            double a = contourArea(contours.get(i), false);
            if(a > largest_area) {
                largest_area = a;
                // Store the index of largest contour
                if(isContourSquare(contours.get(i))) {
                    largest_contour_index = i;
                    bounding_rect = boundingRect(contours.get(i));
                }
            }
        }

        Scalar color = new Scalar(255, 255, 255); //white

        //Draw the contour and rectangle
        if(bounding_rect != null) {
            List<MatOfInt> hull = new ArrayList<MatOfInt>();
            for(int j = 0; j < contours.size(); j++) {
                hull.add(new MatOfInt());
            }

            for(int j = 0; j < contours.size(); j++) {
                Imgproc.convexHull(contours.get(j), hull.get(j));
            }
            Imgproc.drawContours(mat, contours, largest_contour_index, color, Core.FILLED, 8, hierarchy, 0, new Point(0, 0));
            Imgproc.rectangle(rgb, new Point(bounding_rect.x, bounding_rect.y), new Point(bounding_rect.x + bounding_rect.width, bounding_rect.y + bounding_rect.height), new Scalar(255, 255, 255), Core.FILLED);
        }

        return rgb;
    }

    private static boolean isContourSquare(MatOfPoint thisContour) {
        MatOfPoint2f thisContour2f = new MatOfPoint2f();
        MatOfPoint approxContour = new MatOfPoint();
        MatOfPoint2f approxContour2f = new MatOfPoint2f();
        thisContour.convertTo(thisContour2f, CvType.CV_32FC2);

        Imgproc.approxPolyDP(thisContour2f, approxContour2f, 25, true);

        approxContour2f.convertTo(approxContour, CvType.CV_32S);
        Log.i("OPENCV", String.valueOf(approxContour.size().height));

        return approxContour.size().height < 100;
    }
}