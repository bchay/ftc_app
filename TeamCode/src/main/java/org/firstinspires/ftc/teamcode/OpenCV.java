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
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import static org.opencv.imgproc.Imgproc.COLOR_BGR2GRAY;
import static org.opencv.imgproc.Imgproc.COLOR_BGR2HSV;

@Autonomous(name = "OpenCV", group = "Test Code")
public class OpenCV extends LinearOpMode implements CameraBridgeViewBase.CvCameraViewListener2 {
    private OpenCVView mOpenCvCameraView;
    private Mat mat;
    private Mat rgb;
    private Mat gray;
    private Mat resized;

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

        mOpenCvCameraView = (OpenCVView) ((Activity) hardwareMap.appContext).findViewById(R.id.surfaceView);
        //mOpenCvCameraView.setVisibility(CameraBridgeViewBase.VISIBLE);
        mOpenCvCameraView.setCameraIndex(CameraBridgeViewBase.CAMERA_ID_BACK);
        mOpenCvCameraView.setCvCameraViewListener(this);

        //mOpenCvCameraView.setMaxFrameSize(64, 64);

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
        rgb = new Mat(height, width, CvType.CV_8UC4);
        gray = new Mat(height, width, CvType.CV_8UC1);
        resized = new Mat(height, width, CvType.CV_8UC4);
    }

    public void onCameraViewStopped() {
        mat.release();
        rgb.release();
        gray.release();
        resized.release();
    }

    public Mat onCameraFrame(CameraBridgeViewBase.CvCameraViewFrame inputFrame) {
        mat.release();
        rgb.release();
        gray.release();
        resized.release();

        rgb = inputFrame.rgba();

        Imgproc.cvtColor(rgb, mat, COLOR_BGR2HSV);
        Imgproc.cvtColor(rgb, gray, COLOR_BGR2GRAY);

        //Blue Code - TODO
        //Scalar min = new Scalar(100, 50, 50);
        //Scalar max = new Scalar(140, 255, 255);

        //Red Code - Black box
        Scalar min = new Scalar(220/2, 190, 80);
        Scalar max = new Scalar(260, 255, 255);

        Core.inRange(mat, min, max, rgb);

        //Imgproc.resize(rgb, resized, resized.size());

        //return resized;
        return rgb;
    }
}