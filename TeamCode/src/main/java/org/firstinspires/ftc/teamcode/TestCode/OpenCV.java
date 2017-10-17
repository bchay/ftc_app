package org.firstinspires.ftc.teamcode.TestCode;

import android.app.Activity;
import android.graphics.Bitmap;
import android.os.Environment;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.R;
import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.CvException;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.Arrays;
import java.util.Date;
import java.util.concurrent.Exchanger;

import static org.opencv.imgproc.Imgproc.COLOR_BGR2GRAY;
import static org.opencv.imgproc.Imgproc.COLOR_BGR2HSV;
import static org.opencv.imgproc.Imgproc.COLOR_GRAY2RGB;
import static org.opencv.imgproc.Imgproc.COLOR_HSV2RGB;
import static org.opencv.imgproc.Imgproc.COLOR_RGB2HSV;
import static org.opencv.imgproc.Imgproc.MORPH_RECT;
import static org.opencv.imgproc.Imgproc.contourArea;

@Disabled
@Autonomous(name = "OpenCV", group = "Test Code")
public class OpenCV extends LinearOpMode implements CameraBridgeViewBase.CvCameraViewListener2 {
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
        Imgproc.cvtColor(rgb, hsv, COLOR_RGB2HSV);

        String color = "red";
        if(color.equals("blue")) {
            Scalar min = new Scalar(100, 20, 20); //Hue is 0 - 179
            Scalar max = new Scalar(130, 255, 255);
            Core.inRange(hsv, min, max, gray);

            /*
            211, 44, 32
            231, 100, 71
             */

        } else {
            Mat lowerRed = new Mat(); //Red color loops through cylinder
            Core.inRange(hsv, new Scalar(0, 247, 76), new Scalar(11, 255, 138), lowerRed); //0, 68, 30 /// 11, 100, 93)

            Mat upperRed = new Mat();
            Core.inRange(hsv, new Scalar(172, 189, 104), new Scalar(179, 255, 188), upperRed); //345, 84, 41 /// 359, 100, 74

            Core.bitwise_or(lowerRed, upperRed, gray); //Bitwise or combines the ranges
        }
        Mat smallMat = new Mat();

        double desiredWidth = 128;
        double imageWidth = rgb.width();
        double imageHeight = rgb.height();
        double divideFactor = desiredWidth / imageWidth;

        Imgproc.resize(gray, smallMat, new org.opencv.core.Size((int) (imageWidth * divideFactor), (int) (imageHeight * divideFactor)));
        //96 * 128 (w * h)

        Core.transpose(smallMat, smallMat);
        Core.flip(smallMat, smallMat, 1);

        Log.i("OPENCV SIZE", String.valueOf(smallMat.cols()) + " " + String.valueOf(smallMat.rows()));
        Bitmap bmp = null;

        try {
            bmp = Bitmap.createBitmap(smallMat.cols(), smallMat.rows(), Bitmap.Config.ARGB_8888);
            Utils.matToBitmap(smallMat, bmp);
        } catch (CvException e) {
            e.printStackTrace();
        }

        smallMat.release();

        FileOutputStream out = null;

        String filename = new Date() + ".png";

        File dir = new File(Environment.getExternalStorageDirectory() + "/FTC Center Vortex");
        Log.i("Directory: ", dir.getAbsolutePath());
        boolean success = true;

        if (!dir.exists()) {
            success = dir.mkdir();
        }

        if (success) {
            File dest = new File(dir, filename);

            try {
                out = new FileOutputStream(dest);
                bmp.compress(Bitmap.CompressFormat.PNG, 100, out);
            } catch (Exception e) {
                e.printStackTrace();
            } finally {
                try {
                    if (out != null) {
                        out.close();
                    }
                } catch (IOException e) {
                    e.printStackTrace();
                }
            }
        } else {
            Log.i("SUCCESS", String.valueOf(success));
        }

        return gray;
    }
}

/*

345, 90, 41
359, 100, 74

0, 97, 30
11, 100, 54
 */