package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Subsystems.Extendo.ExtendoStates.BASKET;
import static org.firstinspires.ftc.teamcode.Subsystems.Extendo.ExtendoStates.SPECIMEN;
import static org.firstinspires.ftc.teamcode.Subsystems.Extendo.ExtendoStates.ZERO;
import static org.firstinspires.ftc.teamcode.Subsystems.Lift.AUTON_POS;
import static org.firstinspires.ftc.teamcode.Subsystems.Lift.LiftStates.AUTON;
import static org.firstinspires.ftc.teamcode.Subsystems.Lift.LiftStates.AUTON_POS_LOW;
import static org.firstinspires.ftc.teamcode.Subsystems.Lift.LiftStates.DOWN;
import static org.firstinspires.ftc.teamcode.Subsystems.Lift.LiftStates.POS1;
import static org.firstinspires.ftc.teamcode.Subsystems.Lift.POS1_POS;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.ArmGroup;
import org.firstinspires.ftc.teamcode.Subsystems.Claw;
import org.firstinspires.ftc.teamcode.Subsystems.Extendo;
//import org.firstinspires.ftc.teamcode.Subsystems.Lift;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
//import org.firstinspires.ftc.teamcode.Subsystems.ClawRotate;
import org.firstinspires.ftc.teamcode.Subsystems.Lift;
//import org.firstinspires.ftc.teamcode.Subsystems.Rotate;

@Config
@TeleOp
public class TeleOpp extends LinearOpMode {



    private Drivetrain drive = new Drivetrain();
//    private Lift lift = new Lift();
    private Extendo extendo = new Extendo();
//    private ClawRotate clawrotate = new ClawRotate();
    private Claw claw = new Claw();
//    private Rotate rotate = new Rotate();
    private ArmGroup arm = new ArmGroup();

    public static double f = 0.00;
    public static int target = 0;


    private final double ticks_in_degree = 1425.1 / 360.0;
    // private Servo claw;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        drive.init(hardwareMap);
//       lift.init(hardwareMap);
        extendo.init(hardwareMap);
//        clawrotate.init(hardwareMap);
       claw.init(hardwareMap);
       arm.init(hardwareMap);
//       rotate.init(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {

            extendo.loop();
            if (gamepad1.dpad_up) {
                extendo.update(BASKET);
            } if (gamepad1.right_trigger > 0){
                extendo.update(SPECIMEN);
            } if (gamepad1.left_trigger > 0){
                extendo.update(ZERO);
            }

//            lift.loop();
//            if (gamepad1.right_bumper) {
//                lift.update(POS1);
//            } if (gamepad1.left_bumper){
//                lift.update(AUTON_POS_LOW);
//            } if (gamepad1.dpad_up){
//                lift.update(DOWN);
//            }

            double power = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;

            drive.move(power, strafe, turn);
//
//            extendo.extend(gamepad1.right_trigger, gamepad1.left_trigger);

//            rotate.move(gamepad1.right_bumper, gamepad1.left_bumper);
//
            claw.move(gamepad1.a, gamepad1.x);

            arm.move(gamepad1.right_bumper, gamepad1.left_bumper, gamepad1.y, gamepad1.b);

//            if(gamepad1.b) { // left
//                clawrotate.rotate(true, false);
//            } else if(gamepad1.y) { // right
//                clawrotate.rotate(false, true);
//            }

            telemetry.addData("Claw position", claw.getPosition());
//            telemetry.addData("rotate position", rotate.getPosition());
            telemetry.addData("Power", power);
            telemetry.addData("Strafe", strafe);
            telemetry.addData("Turn", turn);
            telemetry.addData("a pressed?", gamepad1.a);
            telemetry.addData("pressed", gamepad1.right_bumper);
//           telemetry.addData("clawRotate", clawrotate.getPosition());

//           telemetry.addData("liftpos", lift.position());
//           telemetry.addData("claw pos", clawrotate.getPosition());
//           telemetry.addData("lift pos", lift.position());
            telemetry.addData("drive pos", drive.getPosition());
           telemetry.update();


        }
    }
}