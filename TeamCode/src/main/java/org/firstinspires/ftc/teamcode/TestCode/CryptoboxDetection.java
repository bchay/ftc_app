package org.firstinspires.ftc.teamcode.TestCode;

import android.graphics.Bitmap;

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
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

@TeleOp(name = "CV Cryptobox Detection", group = "Test Code")
public class CryptoboxDetection extends LinearOpMode {
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
                    detectCryptobox(bitmap);
                    break;
                }
            }
        } catch(InterruptedException e) {
            e.printStackTrace();
        } finally {
            if (frame != null) frame.close();
        }
    }

    void detectCryptobox(Bitmap bitmap) {
        Mat mat = new Mat();
        Mat rotated = new Mat();
        Mat processed = new Mat();
        Mat red1 = new Mat();
        Mat red2 = new Mat();

        Utils.bitmapToMat(bitmap, mat);
        Core.rotate(mat, rotated, Core.ROTATE_90_CLOCKWISE);
        Imgproc.blur(rotated, processed, new Size(10, 10));
        Imgproc.cvtColor(processed, processed, Imgproc.COLOR_RGB2HSV);

        //BLUE
        Core.inRange(processed, new Scalar(67, 80, 40), new Scalar(135, 255, 137), processed);

        //RED
        //Core.inRange(processed, new Scalar(0, 117, 115), new Scalar(6, 255, 255), red1);
        //Core.inRange(processed, new Scalar(114, 209, 106), new Scalar(180, 255, 230), red2);
        //Core.bitwise_or(red1, red2, processed);

        Imgproc.erode(processed, processed, new Mat(), new Point(-1, -1), 8, Core.BORDER_CONSTANT, new Scalar(-1));
        Imgproc.dilate(processed, processed, new Mat(), new Point(-1, -1), 10, Core.BORDER_CONSTANT, new Scalar(-1));

        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(processed, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        //Combine contours
        ArrayList<Rect> boundingRects = new ArrayList<>();

        for(int i = 0; i < contours.size(); i++) {
            if(!opModeIsActive()) break;

            Rect rect = Imgproc.boundingRect(contours.get(i));

            //Remove contour if it is roughly square - ie. jewel
            //All detected contours should have a height that is larger than the width - vertical rectangle
            if((double) rect.height / (double) rect.width > 1.4) boundingRects.add(rect);
        }

        ArrayList<Rect> columns = new ArrayList<>();

        for(int i = 0; i < boundingRects.size(); i++) {
            //Only add to list if it is a unique X coordinate
            boolean addToList = true;

            //Remove contours if they have similar X values
            //Multiple contours for the same column are detected because they are separated by the white tape line
            for(int j = 0; j < columns.size(); j++) {
                if(Math.abs(boundingRects.get(i).x - columns.get(j).x) < 100) {
                    addToList = false;
                    break;
                }
            }

            if(addToList) {
                columns.add(boundingRects.get(i));
            }
        }

        //Columns now contains a list of the cryptobox columns

        //Sort columns to determine distance between adjacent columns
        Collections.sort(columns, new Comparator<Rect>() {
            @Override
            public int compare(Rect lhs, Rect rhs) {
                return lhs.x > rhs.x ? 1 : -1;
            }
        });

        double avg = 0;

        for(int i = 0; i < columns.size() - 1; i++) {
            avg += columns.get(i + 1).x - columns.get(i).x;
        }

        avg /= (double) (columns.size() - 1);

        /*
        //Save Image to Gallery
        Bitmap bm = Bitmap.createBitmap(rotated.width(), rotated.height(), Bitmap.Config.RGB_565);
        Utils.matToBitmap(rotated, bm);
        MediaStore.Images.Media.insertImage(hardwareMap.appContext.getContentResolver(), bm, "OPENCV Mat", "OpenCV");
        */


        telemetry.addData("Distance", 3963.51 / avg + 0.0795029);
        telemetry.addData("Avg", avg);
        telemetry.update();

        mat.release();
        rotated.release();
        processed.release();
        red1.release();
        red2.release();
    }
}
