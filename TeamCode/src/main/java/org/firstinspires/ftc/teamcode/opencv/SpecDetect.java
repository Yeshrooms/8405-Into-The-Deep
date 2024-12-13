package org.firstinspires.ftc.teamcode.opencv;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
// drwaing fun
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;


import java.util.ArrayList;
import java.util.List;

public class SpecDetect implements VisionProcessor {

    private Telemetry telemetry;
    private RotatedRect closestBlock;
    private Double angle; // of closest block

    private enum DetectionMode {
        ALL,
        YELLOW_BLUE,
        YELLOW_RED,
        YELLOW_ONLY,
    }

    private DetectionMode detectionMode = DetectionMode.ALL;

    // matrices for storing images
    private Mat hsvImage; // Holds the converted image in HSV color space
    private Mat redMask1, redMask2, yellowMask, blueMask, redMask, combinedMask, gray, binary, distTransform, sureFg;

    private double size; // area of closest box



    public SpecDetect(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        hsvImage = new Mat();
        redMask1 = new Mat();
        redMask2 = new Mat(); // red just like taht
        yellowMask = new Mat();
        blueMask = new Mat();
        redMask = new Mat();
        combinedMask = new Mat();
        gray = new Mat();
        binary = new Mat();
        distTransform = new Mat(); // adjust for distance
        sureFg = new Mat();
    }

    @Override
    public Object processFrame(Mat inputFrame, long time) {
        // time in nanos
        Mat bgr = new Mat();
        if (inputFrame.channels() == 4) {
            // rgba (4 channel) to gbrSpecDetect
            Imgproc.cvtColor(inputFrame, bgr, Imgproc.COLOR_RGBA2BGR);
        } else {
            bgr = inputFrame;
        }

        Spec<RotatedRect, Double> result = detectBlocks(bgr);
        closestBlock = result.first;
        angle = result.second;

        //drawOutlineAndAngle(bgr, closestBlock, angle);
        // dont call until drawonframe

        double distanceX = 0.0;
        double distanceY = 0.0;
        // from center of image
        // scuffed calculations
        if (closestBlock != null) {
            double centerX = inputFrame.width() / 2.0;
            double centerY = inputFrame.height() / 2.0;
            distanceX = closestBlock.center.x - centerX;
            distanceY = closestBlock.center.y - centerY;

            telemetry.addData("Distance X", String.format("%.2f pixels", distanceX));
            telemetry.addData("Distance Y", String.format("%.2f pixels", distanceY));
        }
        if (closestBlock != null) {
            telemetry.addData("Closest Block Center", String.format("(%.2f, %.2f)", closestBlock.center.x, closestBlock.center.y));
            if (angle != null) {
                telemetry.addData("Angle", String.format("%.2f degrees", angle));
            }
        } else {
            telemetry.addData("No blocks detected", "");
        }
        if (closestBlock != null) {
            size = closestBlock.size.area();
        } else {
            size = 0.0;
        }
        telemetry.addData("Size", size);
        telemetry.update();
        releaseMats(); // free memory
        return bgr; // display
    }

    // draw outline+angle of the block
    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        drawOutlineAndAngle(canvas, closestBlock, angle, scaleBmpPxToCanvasPx);
    }

    private Spec<RotatedRect, Double> detectBlocks(Mat image) {
        // reset mats to initial
        hsvImage.setTo(new Scalar(0));
        redMask1.setTo(new Scalar(0));
        redMask2.setTo(new Scalar(0));
        yellowMask.setTo(new Scalar(0));
        blueMask.setTo(new Scalar(0));
        redMask.setTo(new Scalar(0));
        combinedMask.setTo(new Scalar(0));
        gray.setTo(new Scalar(0));
        binary.setTo(new Scalar(0));
        distTransform.setTo(new Scalar(0));
        sureFg.setTo(new Scalar(0));


        Imgproc.cvtColor(image, hsvImage, Imgproc.COLOR_BGR2HSV);

        // tune these!!!! color ranges
        Scalar lowerRed1 = new Scalar(0.0, 100.0, 100.0);
        Scalar upperRed1 = new Scalar(20.0, 255.0, 255.0);
        Scalar lowerRed2 = new Scalar(150.0, 100.0, 100.0);
        Scalar upperRed2 = new Scalar(179.0, 255.0, 255.0);
        Scalar lowerYellow = new Scalar(22.0, 100.0, 100.0);
        Scalar upperYellow = new Scalar(38.0, 255.0, 255.0);
        Scalar lowerBlue = new Scalar(100.0, 100.0, 100.0);
        Scalar upperBlue = new Scalar(120.0, 255.0, 255.0);

        Mat redMask1 = new Mat();
        Mat redMask2 = new Mat();
        Mat yellowMask = new Mat();
        Mat blueMask = new Mat();
        Core.inRange(hsvImage, lowerRed1, upperRed1, redMask1);
        Core.inRange(hsvImage, lowerRed2, upperRed2, redMask2);
        Core.inRange(hsvImage, lowerYellow, upperYellow, yellowMask);
        Core.inRange(hsvImage, lowerBlue, upperBlue, blueMask);

        // combine red masks
        Mat redMask = new Mat();
        Core.bitwise_or(redMask1, redMask2, redMask);

        // combination based on mode
        Mat combinedMask = new Mat();
        switch (detectionMode) {
            case ALL:
                Core.bitwise_or(redMask, yellowMask, combinedMask);
                Core.bitwise_or(combinedMask, blueMask, combinedMask);
                break;
            case YELLOW_BLUE:
                Core.bitwise_or(yellowMask, blueMask, combinedMask);
                break;
            case YELLOW_RED:
                Core.bitwise_or(yellowMask, redMask, combinedMask);
                break;
            case YELLOW_ONLY:
                combinedMask = yellowMask;
                break;
        }

        // convert to gray
        Mat gray = new Mat();
        if (combinedMask.channels() > 1) {
            Imgproc.cvtColor(combinedMask, gray, Imgproc.COLOR_BGR2GRAY);
        } else {
            gray = combinedMask;
        }

        // binary image (fore vs back ground)
        Mat binary = new Mat();
        Imgproc.threshold(gray, binary, 1, 255, Imgproc.THRESH_BINARY);

        // calculate distances
        Mat distTransform = new Mat();
        Imgproc.distanceTransform(binary, distTransform, Imgproc.DIST_L2, 3);
        Core.normalize(distTransform, distTransform, 0.0, 1.0, Core.NORM_MINMAX);

        // determine which are part of foreground
        Mat sureFg = new Mat();
        Imgproc.threshold(distTransform, sureFg, 0.4, 1.0, Imgproc.THRESH_BINARY);
        sureFg.convertTo(sureFg, CvType.CV_8UC1);

        // find contour of seprated blocks
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(sureFg, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        // Fclosest to the center
        RotatedRect closestBlock = null;
        double closestDistance = Double.MAX_VALUE;
        double centerX = image.width() / 2.0;
        double centerY = image.height() / 2.0;

        for (MatOfPoint contour : contours) {
            RotatedRect rect = Imgproc.minAreaRect(new MatOfPoint2f(contour.toArray()));
            double distance = Math.sqrt(Math.pow(rect.center.x - centerX, 2.0) + Math.pow(rect.center.y - centerY, 2.0));

            if (distance < closestDistance) {
                closestDistance = distance;
                closestBlock = rect;
            }
        }
        // angle calculation
        Double angle = null;
        if (closestBlock != null) {
            angle = closestBlock.angle;
            if (closestBlock.size.width < closestBlock.size.height) {
                angle += 90;
            }
        }

        return new Spec<>(closestBlock, angle);
    }

    public void setDetectionMode(DetectionMode mode) {
        detectionMode = mode;
    }

    // drawing fun
    private void drawOutlineAndAngle(Canvas canvas, RotatedRect block, Double angle, float s) {
        if (block != null) {
            Point[] points = new Point[4];
            block.points(points);

            Paint linePaint = new Paint();
            linePaint.setColor(Color.GREEN);
            linePaint.setStrokeWidth(5);

            for (int i = 0; i < 4; i++) {
                canvas.drawLine((float) points[i].x * s, (float) points[i].y * s,
                        (float) points[(i + 1) % 4].x * s, (float) points[(i + 1) % 4].y * s,
                        linePaint);
            }

            if (angle != null) {
                Paint textPaint = new Paint();
                textPaint.setColor(Color.GREEN);
                textPaint.setTextSize(50);

                String text = String.format("%.2f degrees", angle);
                canvas.drawText(text, (float) (block.center.x - 150) * s, (float) (block.center.y - 50) * s, textPaint);
            }
        }
    }

    public RotatedRect getClosestBlock() {
        return closestBlock;
    }

    public Double getAngle() {
        if (angle != null) {
            return angle;
        } else {
            angle = (double) 0;
            return angle;
        }
    }

    public Double getSize() {
        return size;
    }

    public void update() {
        telemetry.update();
    }

    public static class Spec<F, U> {
        public F first;
        public U second;

        public Spec(F first, U second) {
            this.first = first;
            this.second = second;
        }
    }
    private void releaseMats() {
        hsvImage.release();
        redMask1.release();
        redMask2.release();
        yellowMask.release();
        blueMask.release();
        redMask.release();
        combinedMask.release();
        gray.release();
        binary.release();
        distTransform.release();
        sureFg.release();
    }
}