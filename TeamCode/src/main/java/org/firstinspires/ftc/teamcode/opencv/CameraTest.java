package org.firstinspires.ftc.teamcode.opencv;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.opencv.core.Mat;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Camera Feed Display", group = "Vision")
public class CameraFeedDisplay extends OpMode {

    private OpenCvCamera webcam;

    @Override
    public void init() {
        telemetry.addLine("Initializing...");
        telemetry.update();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        webcam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        webcam.setPipeline(new OpenCvCamera.Pipeline() {
            @Override
            public Mat processFrame(Mat inputFrame) {
                return inputFrame;
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
