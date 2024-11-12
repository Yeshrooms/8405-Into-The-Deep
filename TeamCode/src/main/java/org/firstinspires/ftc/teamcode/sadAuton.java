package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

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
//import org.firstinspires.ftc.teamcode.Subsystems.Lift;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Subsystems.GoBildaPinpointDriver;
@Config

@Autonomous
public class sadAuton extends LinearOpMode {

    private PIDController linearController;
    private PIDController angularController;

    public double lp= 0, li = 0, ld = 0;
    public double ap= 0, ai = 0, ad = 0;

    GoBildaPinpointDriver odo;
    double oldTime = 0;


    @Override
    public void runOpMode() throws InterruptedException {

        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");

        odo.setOffsets(-84.0, -168.0); //these are tuned for 3110-0002-0001 Product Insight #1

        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        odo.resetPosAndIMU();



        Drivetrain drive = new Drivetrain();
//        Lift lift = new Lift();
//        Claw claw = new Claw();
        drive.init(hardwareMap);
        odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");

        waitForStart();

        /* reset x and y values to 0
        reset heading to 0
        pid move forward with angle correction to point
        while lift move up and extend
        while (error > CONST && other stuff finished){
        float pow = pid.calc(error);
        move(error0;
        }
         */

    }

}
