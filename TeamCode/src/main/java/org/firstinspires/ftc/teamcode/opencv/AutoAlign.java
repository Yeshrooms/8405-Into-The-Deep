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

    public static class CameraProcessor implements VisionProcessor, CameraStreamSource {
        private final AtomicReference<Bitmap> lastFrame = new AtomicReference<>(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565));

        @Override
        public void init(int width, int height, CameraCalibration calibration) {
            // match camera resolution, RGB_565 = 16 bit
            lastFrame.set(Bitmap.createBitmap(width, height, Bitmap.Config.RGB_565));
        }

        @Override
        public Object processFrame(Mat frame, long captureTimeNanos) {
            Bitmap bitmap = Bitmap.createBitmap(frame.width(), frame.height(), Bitmap.Config.RGB_565);
            Utils.matToBitmap(frame, bitmap);
            lastFrame.set(bitmap);
            return null;
        }

        @Override
        public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight,
                                float scaleBmpPxToCanvasPx, float scaleCanvasDensity,
                                Object userContext) {
            // do nothing
        }

        @Override
        public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation) {
            // gives latest frame to dashboard
            continuation.dispatch(bitmapConsumer -> bitmapConsumer.accept(lastFrame.get()));
        }

    }

    @Override
    public void runOpMode() {
        servo = hardwareMap.get(Servo.class, "servo");
        SpecDetect blockDetector = new SpecDetect(telemetry);
        CameraProcessor cameraStreamer = new CameraProcessor();
        AnalogInput wristSensor = hardwareMap.get(AnalogInput.class, "wristSensor");
        VisionPortal visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(blockDetector)
                .addProcessor(cameraStreamer)
                .build();
        double wristPos = 0.5;
        FtcDashboard.getInstance().startCameraStream(cameraStreamer, 0);

        waitForStart();

        while (opModeIsActive()) {
            RotatedRect detectedBlock = blockDetector.getClosestBlock();
            Double blockAngle = blockDetector.getAngle();
            if (blockDetector.getSize() > 5000) {
                if (90 < blockAngle && blockAngle < 180) {
                    double adjustment = 90 - (blockAngle - 90);
                    adjustment = adjustment / 90 / 20;
                    wristPos -= adjustment;
                }
                if (0 < blockAngle && blockAngle < 90) {
                    double adjustment = blockAngle / 90 / 20;
                    wristPos += adjustment;
                }
            }
//            servo.setwristPosition(wristPos); TO DO LATER ONCE WE GET CLAW SERVO
            blockDetector.update();

            sleep(30);
        }
    }
}
