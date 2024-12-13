package org.firstinspires.ftc.teamcode.opencv;

import android.graphics.Canvas;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Camera Feed Display", group = "Vision")
public class CameraTest extends OpMode {

    private OpenCvCamera webcam;
    private SpecDetect specDetect;

    @Override
    public void init() {
        telemetry.addLine("Initializing...");
        telemetry.update();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        webcam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"),
                cameraMonitorViewId);

        specDetect = new SpecDetect(telemetry);

        webcam.setPipeline(new VisionProcessor() {
            @Override
            public Mat processFrame(Mat inputFrame, long time) {
                return (Mat) specDetect.processFrame(inputFrame, time);
            }

            @Override
            public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
                specDetect.onDrawFrame(canvas, onscreenWidth, onscreenHeight, scaleBmpPxToCanvasPx, scaleCanvasDensity, userContext);
            }

            @Override
            public void init(int width, int height, CameraCalibration calibration) {
                specDetect.init(width, height, calibration);
            }
        });
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
                telemetry.addLine("Camera opened and streaming started.");
                telemetry.update();
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addLine("Error opening camera: " + errorCode);
                telemetry.update();
            }
        });
    }

    @Override
    public void loop() {
        specDetect.update();
    }

    @Override
    public void stop() {
        if (webcam != null) {
            webcam.stopStreaming();
            webcam.closeCameraDevice();
        }
        telemetry.addLine("Camera feed stopped.");
        telemetry.update();
    }
}

