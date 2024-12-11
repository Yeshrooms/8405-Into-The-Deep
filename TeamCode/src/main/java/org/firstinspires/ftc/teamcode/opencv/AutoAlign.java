package org.firstinspires.ftc.teamcode.vision;

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
import org.firstinspires.ftc.teamcode.vision.SpecDetect;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.android.Utils;
import org.opencv.core.Mat;
import org.opencv.core.RotatedRect;

import java.util.concurrent.atomic.AtomicReference;

@Autonomous
public class AutoAlign extends LinearOpMode {

    public Servo armServo;

    @Override
    public void runOpMode() {
        armServo = hardwareMap.get(Servo.class, "armServo");
        SpecDetect blockFinder = new SpecDetect(telemetry);
        CameraProcessor cameraViewer = new CameraProcessor();
        AnalogInput wristSensor = hardwareMap.get(AnalogInput.class, "wristSensor");
        VisionPortal visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(blockFinder)
                .addProcessor(cameraViewer)
                .build();

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

            armServo.setPosition(armPosition);
            blockFinder.updateTelemetry();
            sleep(30);
        }
    }

    public static class CameraProcessor implements VisionProcessor, CameraStreamSource {

        private final AtomicReference<Bitmap> recentFrame = new AtomicReference<>(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565));

        @Override
        public void init(int width, int height, CameraCalibration calibration) {
            recentFrame.set(Bitmap.createBitmap(width, height, Bitmap.Config.RGB_565));
        }

        @Override
        public Object processFrame(Mat frame, long captureTimeNanos) {
            Bitmap bitmap = Bitmap.createBitmap(frame.width(), frame.height(), Bitmap.Config.RGB_565);
            Utils.matToBitmap(frame, bitmap);
            recentFrame.set(bitmap);
            return null;
        }

        @Override
        public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight,
                                float scaleBmpPxToCanvasPx, float scaleCanvasDensity,
                                Object userContext) {
            // Do nothing
        }

        @Override
        public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation) {
            continuation.dispatch(bitmapConsumer -> bitmapConsumer.accept(recentFrame.get()));
        }

    }
}