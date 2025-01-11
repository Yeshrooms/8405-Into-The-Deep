//package org.firstinspires.ftc.teamcode;
//
//import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
//import static org.firstinspires.ftc.teamcode.Subsystems.Drivetrain.DriveStates.AWAY_BAR_1;
//import static org.firstinspires.ftc.teamcode.Subsystems.Drivetrain.DriveStates.TO_BAR_1;
//import static org.firstinspires.ftc.teamcode.Subsystems.Lift.LiftStates.AUTON;
//import static org.firstinspires.ftc.teamcode.Subsystems.Lift.LiftStates.DOWN;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.arcrobotics.ftclib.controller.PIDController;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
//import org.firstinspires.ftc.teamcode.Subsystems.Claw;
//import org.firstinspires.ftc.teamcode.Subsystems.ClawRotate;
//import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
//import org.firstinspires.ftc.teamcode.Subsystems.Extendo;
//import org.firstinspires.ftc.teamcode.Subsystems.GoBildaPinpointDriver;
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
//import org.firstinspires.ftc.teamcode.Subsystems.GoBildaPinpointDriver;
//import org.firstinspires.ftc.teamcode.Subsystems.Lift;
//
//@Config
//@Autonomous
//public class sadAuton2 extends LinearOpMode {
//
//    GoBildaPinpointDriver odo;
//    Drivetrain drive;
//    Lift lift;
//    Extendo extend;
//    ClawRotate clawRotate;
//    Claw claw;
//
//    private int tick = 0;
//
//    public static float out = 0.45F;
//    public static float in = 0;
//    public static int number = 4;
//    public static int parkPoint = 6;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//
//        // Initialize subsystems
//        drive = new Drivetrain();
//        lift = new Lift();
//        extend = new Extendo();
//        clawRotate = new ClawRotate();
//        claw = new Claw();
//        drive.init(hardwareMap);
//        lift.init(hardwareMap);
//        extend.init(hardwareMap);
//        clawRotate.init(hardwareMap);
//        claw.init(hardwareMap);
//
//        waitForStart();
//
//        while (opModeIsActive()) {
//            boolean liftReady = lift.loop();
//            clawRotate.rotateRight();
//
//            // move to bar
//            if (drive.target < number) {
//                if (drive.loop(telemetry) && liftReady) {
//                    lift.update(AUTON);
//                    extend.extend(out, in);
//                    drive.target++;
//
//                    if (drive.target == number) {
//                        out = 0;
//                        extend.extend(out, in);
//                        claw.open();
//                    }
//                }
//            }
//            else if (drive.target == number) {
//                extend.extend(0, 0);
//                lift.update(DOWN);
//                drive.target++;
//            }
//            else if (drive.target == parkPoint) {
//                if (drive.loop(telemetry)) {
//                    drive.target++;
//                }
//            }
//            else {
//
//                drive.setPowers(0.0);
//            }
//        }
//
//        drive.setPowers(0.0);
//    }
//}
