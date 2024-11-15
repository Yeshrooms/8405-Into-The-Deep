package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static org.firstinspires.ftc.teamcode.Subsystems.Drivetrain.DriveStates.AWAY_BAR_1;
import static org.firstinspires.ftc.teamcode.Subsystems.Drivetrain.DriveStates.TO_BAR_1;
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

    GoBildaPinpointDriver odo;
    Drivetrain drive;
//    Lift lift;

    private int tick = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // init
        drive = new Drivetrain();
//        lift = new Lift();
        drive.init(hardwareMap);
//        lift.init(hardwareMap);

        waitForStart();

//        odo.resetPosAndIMU();

        while (opModeIsActive()) {
            if (drive.loop(telemetry)) {
                if (drive.target < drive.points.length){
                    drive.target++;
                }
                else{
                    drive.setPowers(0);
                }
            }
            telemetry.update();

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


