
package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Subsystems.GoBildaPinpointDriver;

import java.util.Locale;


@TeleOp(name="goBILDA® PinPoint Odometry Example", group="Linear OpMode")
//@Disabled

public class SensorGoBildaPinpointExample extends LinearOpMode {

    GoBildaPinpointDriver odo; // Declare OpMode member for the Odometer Computer

    double oldTime = 0;


    @Override
    public void runOpMode() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");

        odo.setOffsets(-84.0, -168.0); //these are tuned for 3110-0002-0001 Product Insight #1

        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        odo.resetPosAndIMU();


        dashboardTelemetry.addData("Status", "Initialized");
        dashboardTelemetry.addData("X offset", odo.getXOffset());
        dashboardTelemetry.addData("Y offset", odo.getYOffset());
        dashboardTelemetry.addData("Device Version Number:", odo.getDeviceVersion());
        dashboardTelemetry.addData("Device Scalar", odo.getYawScalar());
        dashboardTelemetry.update();

        waitForStart();
        resetRuntime();

        while (opModeIsActive()) {

            odo.update();

            if (gamepad1.a){
                odo.resetPosAndIMU(); //resets the position to 0 and recalibrates the IMU
            }

            if (gamepad1.b){
                odo.recalibrateIMU(); //recalibrates the IMU without resetting position
            }

            double newTime = getRuntime();
            double loopTime = newTime-oldTime;
            double frequency = 1/loopTime;
            oldTime = newTime;

            Pose2D pos = odo.getPosition();
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
            dashboardTelemetry.addData("Position", data);

            Pose2D vel = odo.getVelocity();
            String velocity = String.format(Locale.US,"{XVel: %.3f, YVel: %.3f, HVel: %.3f}", vel.getX(DistanceUnit.MM), vel.getY(DistanceUnit.MM), vel.getHeading(AngleUnit.DEGREES));
            dashboardTelemetry.addData("Velocity", velocity);

            dashboardTelemetry.addData("Status", odo.getDeviceStatus());

            dashboardTelemetry.addData("Pinpoint Frequency", odo.getFrequency()); //prints/gets the current refresh rate of the Pinpoint

            dashboardTelemetry.addData("REV Hub Frequency: ", frequency); //prints the control system refresh rate
            dashboardTelemetry.update();

        }
    }}