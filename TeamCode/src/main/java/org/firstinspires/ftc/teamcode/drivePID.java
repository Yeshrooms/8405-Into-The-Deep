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

    public static double p = 0.003, i = 0, d = 0.0;

    private Drivetrain drive = new Drivetrain();

    private Lift lift = new Lift();

    public static int targetDistance = 120;

    public static int targetAngle = 0;

    private GoBildaPinpointDriver odo;
    private PIDController linearController;
    private PIDController angularController;

    public double lp= 0.0, li = 0.0, ld = 0.0;
    public double ap= 0.0, ai = 0.0, ad = 0.0;


    public void setTarget(int targetPos) {
        targetDistance = targetPos;
    }


    @Override
    public void runOpMode() throws InterruptedException {

        lift.init(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");

        // configure
        odo.setOffsets(-84.0, -168.0);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.resetPosAndIMU();

        linearController = new PIDController(lp, li, ld);
        angularController = new PIDController(ap, ai, ad);

        drive.init(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {

            odo.update();
            Pose2D currentPosition = odo.getPosition();
            double currentDistance = currentPosition.getX(DistanceUnit.MM);
            double currentHeading = currentPosition.getHeading(AngleUnit.DEGREES);

            // distance
            double distanceError = targetDistance - currentDistance;
            double forwardPower = linearController.calculate(distanceError);

            // angle
            double angleError = targetAngle - currentHeading;
            double anglePower = angularController.calculate(angleError);

            // maths more like meths
            double leftPower = forwardPower - anglePower;
            double rightPower = forwardPower + anglePower;
            drive.setPowers(leftPower, leftPower, rightPower, rightPower);

            telemetry.addData("Current distance", currentDistance);
            telemetry.addData("Distance error", distanceError);
            telemetry.addData("Current heading", currentHeading);
            telemetry.addData("Heading error", angleError);
            telemetry.addData("Left power", leftPower);
            telemetry.addData("Right power", rightPower);
            telemetry.update();

        }
    }
}