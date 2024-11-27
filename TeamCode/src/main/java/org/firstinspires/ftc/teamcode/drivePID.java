package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.Subsystems.Lift;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;


@Config
@TeleOp
public class drivePID extends LinearOpMode{

    private Drivetrain drive = new Drivetrain();

    public static int targetDistance = 20;

    public static double targetAngle = 0;

    private GoBildaPinpointDriver odo;
    private PIDController linearController;
    private PIDController angularController;

    public static double lp= 0.0305, li = 0.0, ld = 0.0;
    public static double ap= 0.0078, ai = 0.0, ad = 0.0;


    public void setTarget(int targetPos) {
        targetDistance = targetPos;
    }


    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");

        // configure
        odo.setOffsets(-84.0, -168.0);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        odo.resetPosAndIMU();

        linearController = new PIDController(lp, li, ld);
        angularController = new PIDController(ap, ai, ad);

        drive.init(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {

            odo.update();
            Pose2D currentPosition = odo.getPosition();
            double currentDistance = -currentPosition.getY(DistanceUnit.INCH);
            double currentHeading = currentPosition.getHeading(AngleUnit.DEGREES);

            // distance
            double distanceError = targetDistance - currentDistance;
            double forwardPower = linearController.calculate(currentDistance, targetDistance);
            if (forwardPower > 0.6){
                forwardPower = 0.6;
            }
            // angle
            double angleError = targetAngle - currentHeading;
            double anglePower = angularController.calculate(currentHeading, targetAngle);

            // maths more like meths
            double leftPower = forwardPower - anglePower;
            double rightPower = forwardPower + anglePower;

            drive.setPowers(-leftPower, leftPower, rightPower, -rightPower);

            telemetry.addData("Current distance", currentDistance);
            telemetry.addData("Current heading", currentHeading);
            telemetry.addData("X", currentPosition.getX(DistanceUnit.INCH));
            telemetry.addData("Y", currentPosition.getY(DistanceUnit.INCH));
            telemetry.addData("Distance error", distanceError);
            telemetry.addData("Current heading", currentHeading);
            telemetry.addData("Heading error", angleError);
            telemetry.addData("Angle power", anglePower);
            telemetry.addData("Forward power", forwardPower);
            telemetry.addData("Left power", leftPower);
            telemetry.addData("Right power", rightPower);
            telemetry.update();

        }
    }
}