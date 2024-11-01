package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import org.Subsystems.Claw;
import org.Subsystems.Extendo;
import org.Subsystems.Lift;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
@Config
@TeleOp
public class TeleOpp extends LinearOpMode {

    private Drivetrain drive = new Drivetrain();
    private Lift lift = new Lift();
    private Extendo extendo = new Extendo();
    private Claw claw = new Claw();

    public static double f = 0;
    public static int target = 0;



    private final double ticks_in_degree = 1425.1 / 360.0;
    // private Servo claw;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        drive.init(hardwareMap);
        lift.init(hardwareMap);
        extendo.init(hardwareMap);
        // claw = hardwareMap.get(Servo.class, "claw");
        claw.init(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {

            double power = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;

            drive.move(power, strafe, turn);

            double ff = Math.cos(Math.toRadians(drive.getPosition()) / ticks_in_degree)) * f;

            int liftPos = lift.position();

            lift.move(gamepad1.right_bumper, gamepad1.left_bumper, ff);

            extendo.extend(gamepad1.right_trigger, gamepad1.left_trigger);

            claw.move(gamepad1.a, gamepad1.b);

            telemetry.addData("Power", power);
            telemetry.addData("Strafe", strafe);
            telemetry.addData("Turn", turn);
            telemetry.addData("a pressed?", gamepad1.a);
            telemetry.addData("pressed", gamepad1.right_bumper);
//            telemetry.addData("ff", ff);
//            telemetry.addData("liftpos", lift.position());
            telemetry.addData("claw pos", claw.getPosition());
            telemetry.addData("lift pos", lift.position());
            telemetry.addData("drive pos", drive.getPosition());
            telemetry.addData("ff", ff);
            telemetry.update();


        }
    }
}