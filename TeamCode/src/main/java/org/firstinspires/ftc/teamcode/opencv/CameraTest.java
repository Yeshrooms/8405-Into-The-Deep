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

        // Get the camera monitor view ID (needed to display the camera feed on the Driver Station)
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        // Create the webcam instance using EasyOpenCV
        webcam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        // Set the webcam's pipeline (to just show the frame as it is, without processing)
        webcam.setPipeline(new OpenCvCamera.PipelinOvee() {
            @Override
            public Mat processFrame(Mat inputFrame) {
                // No processing, just return the frame
                return inputFrame;
            }
        });

        // Open the camera and start streaming
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
        // Camera feed will automatically be shown on the Driver Station screen.
        // You can use this method to add more logic if needed during the TeleOp period.
    }

    @Override
    public void stop() {
        // Stop the camera when the OpMode is stopped
        if (webcam != null) {
            webcam.stopStreaming();
            webcam.closeCameraDevice();
        }
        telemetry.addLine("Camera feed stopped.");
        telemetry.update();
    }
}
