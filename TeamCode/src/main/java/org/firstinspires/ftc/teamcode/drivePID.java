package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.geometry.Pose2d;
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

    public static double yDistance = 0;
    public static double xDistance = -20;

    public static double norm = 8/5.0;

    public static double targetAngle = 0;

    private GoBildaPinpointDriver odo;
    private PIDController yLinearController;
    private PIDController xLinearController;
    private PIDController angularController;

    public static double lp= 0.021, li = 0.04, ld = 0.0;
    public static double ap= 0.02, ai = 0.0, ad = 0.0;


    public void setTarget(int targetPos) {
        yDistance = targetPos;
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

        yLinearController = new PIDController(lp, li, ld);
        xLinearController = new PIDController(lp, li, ld);
        angularController = new PIDController(ap, ai, ad);

        drive.init(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {

//            odo.update();
//            Pose2D currentPosition = odo.getPosition();
//            double currentDistance = -currentPosition.getY(DistanceUnit.INCH)*norm;
//            double currentHeading = -currentPosition.getHeading(AngleUnit.DEGREES);
//
//            // distance
//            double distanceError = targetDistance - currentDistance;
//            double forwardPower = linearController.calculate(currentDistance, targetDistance);
//            if (forwardPower > 0.6){
//                forwardPower = 0.6;
//            }
//            // angle
//            double angleError = targetAngle - currentHeading;
//            double anglePower = angularController.calculate(currentHeading, targetAngle);
//
//            // maths more like meths
//            double leftPower = forwardPower - anglePower;
//            double rightPower = forwardPower + anglePower;
//
//            drive.setPowers(leftPower, -leftPower, -rightPower, rightPower);
//
//            telemetry.addData("Current distance", currentDistance);
//            telemetry.addData("Current heading", currentHeading);
//            telemetry.addData("X", currentPosition.getX(DistanceUnit.INCH));
//            telemetry.addData("Y", currentPosition.getY(DistanceUnit.INCH));
//            telemetry.addData("Distance error", distanceError);
//            telemetry.addData("Current heading", currentHeading);
//            telemetry.addData("Heading error", angleError);
//            telemetry.addData("Angle power", anglePower);
//            telemetry.addData("Forward power", forwardPower);
//            telemetry.addData("Left power", leftPower);
//            telemetry.addData("Right power", rightPower);
//            telemetry.update();

            odo.update();
            double xTarget = xDistance;
            double yTarget = yDistance;
            double targetAngle = -90.0;
            Pose2D currentPosition = odo.getPosition();
            double yDistance = -currentPosition.getY(DistanceUnit.INCH);
            double xDistance = -currentPosition.getX(DistanceUnit.INCH);
            double currentHeading = currentPosition.getHeading(AngleUnit.RADIANS);

            // distance

            double forwardPower = -yLinearController.calculate(yDistance, yTarget);
            double strafePower = -xLinearController.calculate(xDistance, xTarget);

            // angle
            double anglePower = angularController.calculate(Math.toDegrees(currentHeading), targetAngle);


            // Rotate the movement direction counter to the bot's rotation
            double rotX = forwardPower * Math.cos(currentHeading) - strafePower * Math.sin(currentHeading);
            double rotY = forwardPower * Math.sin(currentHeading) + strafePower * Math.cos(currentHeading);

            rotX = rotX * 1.1;  // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(anglePower), 1);
            double frontLeftPower = (rotY + rotX + anglePower) / denominator;
            double backLeftPower = (rotY - rotX + anglePower) / denominator;
            double frontRightPower = (rotY - rotX - anglePower) / denominator;
            double backRightPower = (rotY + rotX - anglePower) / denominator;

            drive.setPowers(frontLeftPower, -backLeftPower, -frontRightPower, backRightPower);

            boolean atX = Math.abs(xTarget - xDistance) < 4;
            boolean atY = Math.abs(yTarget - yDistance) < 8;
            boolean atHeading = Math.abs(targetAngle - currentHeading) < 10;

            telemetry.addData("xtar", xTarget);
            telemetry.addData("ytar", yTarget);
            telemetry.addData("headingtar", targetAngle);
            telemetry.addData("x", xDistance);
            telemetry.addData("y", yDistance);
            telemetry.addData("heading", currentHeading);
            telemetry.addData("forPow", forwardPower);
            telemetry.addData("strPow", strafePower);
            telemetry.addData("angPow", anglePower);
            telemetry.addData("atXerror", atX);
            telemetry.addData("atYerror", atY);
            telemetry.addData("atHeadingError", atHeading);
//            return atX && atY && atHeading;

        }
    }
}