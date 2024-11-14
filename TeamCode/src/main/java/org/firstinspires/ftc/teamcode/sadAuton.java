package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static org.firstinspires.ftc.teamcode.Subsystems.Lift.LiftStates.AUTON;
import static org.firstinspires.ftc.teamcode.Subsystems.Lift.LiftStates.DOWN;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.firstinspires.ftc.teamcode.Subsystems.Claw;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.GoBildaPinpointDriver;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Subsystems.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.Subsystems.Lift;

@Config
@Autonomous
public class sadAuton extends LinearOpMode {

    private PIDController linearController;
    private PIDController angularController;

    public double lp= 0.0, li = 0.0, ld = 0.0;  
    public double ap= 0.0, ai = 0.0, ad = 0.0;  

    GoBildaPinpointDriver odo;
    Drivetrain drive;
    Lift lift;

    public double targetDistance = 500; // mm
    public double targetAngle = 0; // degrees
    private int tick = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();

//        odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");
//
//        // configure
//        odo.setOffsets(-84.0, -168.0);
//        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
//        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
//        odo.resetPosAndIMU();

        // init
        drive = new Drivetrain();
        lift = new Lift();
        drive.init(hardwareMap);
        lift.init(hardwareMap);
        linearController = new PIDController(lp, li, ld);
        angularController = new PIDController(ap, ai, ad);

        waitForStart();

//        odo.resetPosAndIMU();
        lift.update(AUTON);

        while (opModeIsActive()) {
            int error = lift.loop();
            int driveError = drive.loop();
            tick++;
            if (Math.abs(error) < 8 && ){
                lift.update(DOWN);
            }
            if (tick > 1200){
                lift.update(AUTON);
            }
//            odo.update();
//            Pose2D currentPosition = odo.getPosition();
//            double currentDistance = currentPosition.getX(DistanceUnit.MM);
//            double currentHeading = currentPosition.getHeading(AngleUnit.DEGREES);
//
//            // distance
//            double distanceError = targetDistance - currentDistance;
//            double forwardPower = linearController.calculate(distanceError);
//
//            // angle
//            double angleError = targetAngle - currentHeading;
//            double anglePower = angularController.calculate(angleError);
//
//            // maths more like meths
//            double leftPower = forwardPower - anglePower;
//            double rightPower = forwardPower + anglePower;
//            drive.setPowers(leftPower, leftPower, rightPower, rightPower);
//
//
//            telemetry.addData("Current distance", currentDistance);
//            telemetry.addData("Distance error", distanceError);
//            telemetry.addData("Current heading", currentHeading);
//            telemetry.addData("Heading error", angleError);
//            telemetry.addData("Left power", leftPower);
//            telemetry.addData("Right power", rightPower);
//            telemetry.update();
//
//            if (Math.abs(distanceError) < 5 && Math.abs(angleError) < 1) {
//                break;
//            }
        }
//        drive.setPowers(0.0);
    }


}


