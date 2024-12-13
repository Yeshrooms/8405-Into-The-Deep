package org.firstinspires.ftc.teamcode.opencv;

import android.graphics.Bitmap;
import android.graphics.Canvas;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.opencv.SpecDetect;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.android.Utils;
import org.opencv.core.Mat;
import org.opencv.core.RotatedRect;

import java.util.concurrent.atomic.AtomicReference;

@Autonomous
public class AutoAlign extends LinearOpMode {

    public Servo servo;

    @Override
    public void runOpMode() {
        servo = hardwareMap.get(Servo.class, "armServo");
        SpecDetect blockFinder = new SpecDetect(telemetry);
        CameraStreamProcessor  cameraViewer = new CameraStreamProcessor ();
        AnalogInput wristSensor = hardwareMap.get(AnalogInput.class, "wristSensor");
        VisionPortal visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(blockFinder)
                .addProcessor(cameraViewer)
                .build();
        // vision portal that connects webcam/ vision procesisng pipeline

        double armPosition = 0.5;
        FtcDashboard.getInstance().startCameraStream(cameraViewer, 0);

        waitForStart();

        while (opModeIsActive()) {
            RotatedRect detectedObject = blockFinder.getClosestBlock();
            Double objectAngle = blockFinder.getAngle();

            if (blockFinder.getSize() > 5000) {
                if (90 < objectAngle && objectAngle < 180) {
                    double adjustment = 90 - (objectAngle - 90);
                    adjustment = adjustment / 1800;
                    armPosition -= adjustment;
                }
                if (0 < objectAngle && objectAngle < 90) {
                    double adjustment = objectAngle / 1800;
                    armPosition += adjustment;
                }
            }

            servo.setPosition(armPosition);
            blockFinder.update();
            sleep(30);
        }
    }

    public static class CameraStreamProcessor implements VisionProcessor, CameraStreamSource {
        private final AtomicReference<Bitmap> lastFrame =
                new AtomicReference<>(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565));


        @Override
        public void init(int width, int height, CameraCalibration calibration) {
            lastFrame.set(Bitmap.createBitmap(width, height, Bitmap.Config.RGB_565));
        }
        // blank bitmap to hold camera feed

        @Override
        public Object processFrame(Mat frame, long captureTimeNanos) {
            Bitmap b = Bitmap.createBitmap(frame.width(), frame.height(), Bitmap.Config.RGB_565);
            Utils.matToBitmap(frame, b);
            lastFrame.set(b);
            return null;
        }
        // converts opencv mat frame into bitmap and stores

        @Override
        public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight,
                                float scaleBmpPxToCanvasPx, float scaleCanvasDensity,
                                Object userContext) {
            // do nothing, should be used to draw
        }

        @Override
        public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation) {
            continuation.dispatch(bitmapConsumer -> bitmapConsumer.accept(lastFrame.get()));
        }
        // returns last camera frame as bitmpa

    }
}