package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static org.firstinspires.ftc.teamcode.Subsystems.Drivetrain.DriveStates.AWAY_BAR_1;
import static org.firstinspires.ftc.teamcode.Subsystems.Drivetrain.DriveStates.TO_BAR_1;
import static org.firstinspires.ftc.teamcode.Subsystems.Lift.LiftStates.AUTON;
import static org.firstinspires.ftc.teamcode.Subsystems.Lift.LiftStates.AUTON_DOWN;
import static org.firstinspires.ftc.teamcode.Subsystems.Lift.LiftStates.AUTON_POS_LOW;
import static org.firstinspires.ftc.teamcode.Subsystems.Lift.LiftStates.DOWN;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.firstinspires.ftc.teamcode.Subsystems.ArmGroup;
import org.firstinspires.ftc.teamcode.Subsystems.Claw;
import org.firstinspires.ftc.teamcode.Subsystems.ClawRotate;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.Extendo;
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
    Extendo extend;
//    ClawRotate clawRotate;
    Claw claw;
    ArmGroup arm;

    private int num = 0;
//
//    private int tick = 0;
//
//    public static float out = 0.55F;
//    public static float in = 0.2F;
    public static int number = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // init
        drive = new Drivetrain();

//        lift = new Lift();
        extend = new Extendo();
//        clawRotate = new ClawRotate();
        claw = new Claw();
        arm = new ArmGroup();
        drive.init(hardwareMap);
        arm.init(hardwareMap);
//        clawRotate.init(hardwareMap);
        claw.init(hardwareMap);
        extend.init(hardwareMap);
//        boolean yay = false;
//        int tick = 0;
//        boolean open = false;
//        out = 0.55F;

        waitForStart();


//        odo.resetPosAndIMU();
//
        while (opModeIsActive()) {

            if (num < 2){
                arm.move(true, false, false, false);
            }
            //            if (!yay) {
//                lift.update(AUTON);
//                int liftError = lift.loop();
//                if (liftError < 10) {
//                    yay = drive.loop(telemetry);
//                    extend.extend(1, 0);
//                    clawRotate.rotate(false, true);
//                }
//            } else if (!open) {
//                tick++;
//                lift.loop();
//                extend.extend(0,0.8f);
//                if (tick > 1000){
//                    claw.open();
//                    open = true;
//                    lift.update(DOWN);
//                    drive.target = 1;
//                }
//                telemetry.addData("ticks", tick);
//            } else{
//                lift.loop();
//            }
//            telemetry.addData("ticks", tick);
//            telemetry.addData("lift position", lift.getPosition());
//
//            telemetry.update();
//        }
//            boolean hi = lift.loop();
//            clawRotate.rotateRight();

            if (drive.loop(telemetry) && extend.loop()) {
//                if (drive.target < number+1) {
//                    lift.update(AUTON);
//                }
                if (num < 2) {
//                arm.move(false, false, true,);
                    extend.update(Extendo.ExtendoStates.AUTON);
                    if (drive.target < 1) {
                        drive.target++;
                    }
                    num++;
                }
                if (num == 2){
                    claw.move(true, false);
                    extend.update(Extendo.ExtendoStates.ZERO);
                    arm.initpos();
                }



//                if (drive.target < drive.points.length-1) {
////                    clawRotate.rotateRight();
//                    drive.target++;
//                    if (drive.target == number-1){
//                        extend.extend(out,in);
//                    }
//                    if (drive.target == number){
//                        lift.update(AUTON_DOWN);
//                        extend.extend(0,in);
//                    }
//                    if (drive.target == number+1){
//                        claw.open();
//                        extend.extend(0, 0.2F);
//                        drive.target++;
////                        clawRotate.rotateLeft();
//                        lift.update(DOWN);
//                    }
//                }
            }

//            telemetry.addData("Current distance", currentDistance);
//            telemetry.addData("Distance error", distanceError);
//            telemetry.addData("Current heading", currentHeading);
//            telemetry.addData("Heading error", angleError);
//            telemetry.addData("Left power", leftPower);
//            telemetry.addData("Right power", rightPower);
            telemetry.addData("drive.target", drive.target);
            telemetry.update();
//
//            if (Math.abs(distanceError) < 5 && Math.abs(angleError) < 1) {
//                break;
//            }
        }
//        lift.update(DOWN);
        drive.setPowers(0.0);
    }
}





