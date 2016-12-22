package org.firstinspires.ftc.teamcode;

import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.CameraBridgeViewBase.CvCameraViewFrame;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.android.CameraBridgeViewBase.CvCameraViewListener2;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import android.app.Activity;
import android.hardware.Camera.Size;
import android.os.Environment;
import android.util.Log;
import android.view.Menu;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.text.SimpleDateFormat;
import java.util.Arrays;
import java.util.Date;
import java.util.List;
import java.util.ListIterator;
import java.util.Locale;

import static org.opencv.imgproc.Imgproc.COLOR_BGR2GRAY;
import static org.opencv.imgproc.Imgproc.COLOR_BGR2HSV;
import static org.opencv.imgproc.Imgproc.COLOR_HSV2RGB;
import static org.opencv.imgproc.Imgproc.COLOR_RGB2GRAY;
import static org.opencv.imgproc.Imgproc.COLOR_RGB2HSV;

@Autonomous(name="OpenCV Test")
public class OpenCVTest extends LinearOpMode implements CvCameraViewListener2 {
    private OpenCVView mOpenCvCameraView;
    private List<Size> mResolutionList;
    private Mat rgb;
    private Mat gray;
    private Mat hsv;
    boolean log = true;

    @Override
    public void runOpMode() throws InterruptedException {
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
        mOpenCvCameraView.setCvCameraViewListener(this);

        if (!OpenCVLoader.initDebug()) {
            OpenCVLoader.initAsync(OpenCVLoader.OPENCV_VERSION_3_0_0, hardwareMap.appContext, mLoaderCallback);
        } else {
            mLoaderCallback.onManagerConnected(LoaderCallbackInterface.SUCCESS);
        }

        while (!isStopRequested() && mOpenCvCameraView.getMCamera() == null) {
            idle();
        }


        //Wait for all camera resolutions to load
        while (!isStopRequested() && mOpenCvCameraView.getResolutionList().size() < 9) {
            idle();
        }

        mResolutionList = mOpenCvCameraView.getResolutionList();
        mOpenCvCameraView.setResolution(mResolutionList.get(mResolutionList.size() - 1)); //176 x 144

        telemetry.addData("Start", "");
        telemetry.update();

        waitForStart();
        while (opModeIsActive()) {
            idle();
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

    public Mat onCameraFrame(CvCameraViewFrame inputFrame) {
        if (rgb != null) rgb.release();
        if (hsv != null) hsv.release();
        if (gray != null) gray.release();

        rgb = inputFrame.rgba();

        Imgproc.cvtColor(rgb, hsv, COLOR_RGB2HSV);

        //Blue Code
        Scalar min = new Scalar(90, 110, 110); //Hue is 0 - 179
        Scalar max = new Scalar(140, 255, 255);

        //Red Code
        //Scalar min = new Scalar(0, 150, 150);
        //Scalar max = new Scalar(5, 255, 255);

        Core.inRange(hsv, min, max, gray); //Threshold for alliance color

        Mat m = new Mat();
        Imgproc.resize(gray, m, new org.opencv.core.Size(20, 20));

        byte[] pixels = new byte[m.height() * m.width()];
        m.get(0, 0, pixels);
        Log.i("MAT", Arrays.toString(convertToIntArray(pixels)));

        return gray;
    }

    private static int[] convertToIntArray(byte[] input) {
        int[] ret = new int[input.length];
        for (int i = 0; i < input.length; i++)
        {
            ret[i] = input[i] & 0xff; // Range 0 to 255, not -128 to 127
        }
        return ret;
    }
}