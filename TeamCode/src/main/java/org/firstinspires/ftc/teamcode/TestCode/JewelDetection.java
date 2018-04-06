package org.firstinspires.ftc.teamcode.TestCode;

import android.graphics.Bitmap;
import android.provider.MediaStore;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "CV Jewel Detection", group = "Test Code")
public class JewelDetection extends LinearOpMode {
    VuforiaLocalizer vuforia;
    Image rgb;

    static {
        System.loadLibrary("opencv_java3");
    }

    @Override public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = "AcSmd8X/////AAAAGY0Yqr1v4EYCrKeT+OkWoBQ5q0bkTU0tTzZ+CXBHFr9bALTHzyDlh8FtKKyPmJbF8mtacz2bLRrwvgDcGWGzlqWGdET+qipWSFq1aRJy02jB7rEtnOdJV6mCyzUErz55VFh8DTFW3A373oHemYSnOQL63h88FknvhskKiGckmYIX3vhVpj3QvEpEL4oChVlnmr4AGLfhTyFLQ9C1R7iNxOSP8FAOzDslaDWJ6jT4iaD4E96jNgxhjXaWxmOqRW4MeXXmBESdV45cvxbGAlfwv6cp/zB8b+w8AI4YpwwbGDqPT4nimANFBQ3zAJTlZTDDFeyVbA0bQIWQuIi+ZACA/qloSYaNOYOkelXQ+JTj0dFv";

        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);
        vuforia.setFrameQueueCapacity(4);

        telemetry.addData("Press play", "");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            if(gamepad1.a) {
                processImage();
                sleep(5000);
            }
        }
    }

    private void processImage() {
        VuforiaLocalizer.CloseableFrame frame = null;

        try {
            frame = vuforia.getFrameQueue().take();

            for (int i = 0; i < frame.getNumImages(); i++) {
                Image img = frame.getImage(i);

                if (img.getFormat() == PIXEL_FORMAT.RGB565) {
                    rgb = frame.getImage(i);

                    Bitmap bitmap = Bitmap.createBitmap(rgb.getWidth(), rgb.getHeight(), Bitmap.Config.RGB_565);
                    bitmap.copyPixelsFromBuffer(rgb.getPixels());
                    jewel(bitmap);
                    break;
                }
            }
        } catch(InterruptedException e) {
            e.printStackTrace();
        } finally {
            if (frame != null) frame.close();
        }
    }

    void jewel(Bitmap bitmap) {
        Mat mat = new Mat();
        Mat rotated = new Mat();
        Mat processed = new Mat();
        Mat blue = new Mat();
        Mat red = new Mat();
        Mat red1 = new Mat();
        Mat red2 = new Mat();

        Utils.bitmapToMat(bitmap, mat);
        Core.rotate(mat, rotated, Core.ROTATE_90_CLOCKWISE);
        Imgproc.blur(rotated, processed, new Size(10, 10));
        Imgproc.cvtColor(processed, processed, Imgproc.COLOR_RGB2HSV);

        Core.inRange(processed, new Scalar(84, 177, 89), new Scalar(126, 255, 255), blue);

        Core.inRange(processed, new Scalar(0, 117, 115), new Scalar(6, 255, 255), red1);
        Core.inRange(processed, new Scalar(114, 209, 106), new Scalar(180, 255, 230), red2);
        Core.bitwise_or(red1, red2, red); //Combine upper and lower red hue ranges

        Bitmap bm = Bitmap.createBitmap(rotated.width(), rotated.height(), Bitmap.Config.RGB_565);
        Utils.matToBitmap(blue, bm);
        MediaStore.Images.Media.insertImage(hardwareMap.appContext.getContentResolver(), bm, "OPENCV Mat", "OpenCV");

        bm = Bitmap.createBitmap(rotated.width(), rotated.height(), Bitmap.Config.RGB_565);
        Utils.matToBitmap(red, bm);
        MediaStore.Images.Media.insertImage(hardwareMap.appContext.getContentResolver(), bm, "OPENCV Mat", "OpenCV");

        Imgproc.erode(blue, blue, new Mat(), new Point(-1, -1), 5, Core.BORDER_CONSTANT, new Scalar(-1));
        Imgproc.dilate(blue, blue, new Mat(), new Point(-1, -1), 25, Core.BORDER_CONSTANT, new Scalar(-1));

        Imgproc.erode(red, red, new Mat(), new Point(-1, -1), 5, Core.BORDER_CONSTANT, new Scalar(-1));
        Imgproc.dilate(red, red, new Mat(), new Point(-1, -1), 25, Core.BORDER_CONSTANT, new Scalar(-1));

        List<MatOfPoint> blueContours = new ArrayList<>();
        List<MatOfPoint> redContours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(blue, blueContours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        Imgproc.findContours(red, redContours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        //Get bounding rectangles
        if(blueContours.size() == 1 && redContours.size() == 1) {
            if (Imgproc.boundingRect(blueContours.get(0)).x < Imgproc.boundingRect(redContours.get(0)).x) telemetry.addData("Blue Red", "");
            else telemetry.addData("Red Blue", "");
        } else telemetry.addData("Unknown", "Blue: " + blueContours.size() + ", Red: " + redContours.size());

        telemetry.update();

        bm = Bitmap.createBitmap(rotated.width(), rotated.height(), Bitmap.Config.RGB_565);
        Utils.matToBitmap(blue, bm);
        MediaStore.Images.Media.insertImage(hardwareMap.appContext.getContentResolver(), bm, "OPENCV Mat", "OpenCV");

        bm = Bitmap.createBitmap(rotated.width(), rotated.height(), Bitmap.Config.RGB_565);
        Utils.matToBitmap(red, bm);
        MediaStore.Images.Media.insertImage(hardwareMap.appContext.getContentResolver(), bm, "OPENCV Mat", "OpenCV");

        bm = Bitmap.createBitmap(rotated.width(), rotated.height(), Bitmap.Config.RGB_565);
        Utils.matToBitmap(rotated, bm);
        MediaStore.Images.Media.insertImage(hardwareMap.appContext.getContentResolver(), bm, "OPENCV Mat", "OpenCV");

        mat.release();
        rotated.release();
        processed.release();
        blue.release();
        red.release();
        red1.release();
        red2.release();
    }
}