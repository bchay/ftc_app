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

        while(!isStopRequested() && mOpenCvCameraView.getMCamera() == null) {
            idle();
        }


        //Wait for all camera resolutions to load
        while(!isStopRequested() && mOpenCvCameraView.getResolutionList().size() < 9) {
            idle();
        }

        mResolutionList = mOpenCvCameraView.getResolutionList();
        mOpenCvCameraView.setResolution(mResolutionList.get(mResolutionList.size() - 1)); //176 x 144

        telemetry.addData("Start", "");
        telemetry.update();

        double currentTime = System.currentTimeMillis();
        waitForStart();
        while(opModeIsActive()) {
            if(System.currentTimeMillis() - currentTime > 5000) {
                SimpleDateFormat sdf = new SimpleDateFormat("yyyy-MM-dd_HH-mm-ss", Locale.US);
                String currentDateandTime = sdf.format(new Date());
                String fileName = Environment.getExternalStorageDirectory().getPath() +
                        "/sample_picture_" + currentDateandTime + ".jpg";
                mOpenCvCameraView.takePicture(fileName);
                currentTime = System.currentTimeMillis();
            }
            idle();
        }

        if (mOpenCvCameraView != null) {
            //mOpenCvCameraView.disableView();
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
        rgb.release();
        hsv.release();
        gray.release();

        rgb = inputFrame.rgba();

        Imgproc.cvtColor(rgb, hsv, COLOR_RGB2HSV);
        Imgproc.cvtColor(rgb, gray, COLOR_RGB2GRAY);

        //Blue Code
        Scalar min = new Scalar(90, 110, 110); //Hue is 0 - 179
        Scalar max = new Scalar(140, 255, 255);

        //Red Code
        //Scalar min = new Scalar(0, 150, 150);
        //Scalar max = new Scalar(5, 255, 255);

        Core.inRange(hsv, min, max, gray); //Threshold for alliance color

        return gray;
    }
}