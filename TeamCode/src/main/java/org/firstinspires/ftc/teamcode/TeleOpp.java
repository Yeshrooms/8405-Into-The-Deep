package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
//
import org.Subsystems.Claw;
import org.Subsystems.Extendo;
import org.Subsystems.Lift;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;

@TeleOp
public class TeleOpp extends LinearOpMode {

    private Drivetrain drive = new Drivetrain();
    private Lift lift = new Lift();
    private Extendo extendo = new Extendo();
    private Claw claw = new Claw();

    @Override
    public void runOpMode() {
        drive.init(hardwareMap);
        lift.init(hardwareMap);
        extendo.init(hardwareMap);
        claw.init(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {


            double power = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;

            drive.move(power, strafe, turn);

            lift.move(gamepad1.right_bumper, gamepad1.left_bumper);

            extendo.extend(gamepad1.right_trigger, gamepad1.left_trigger);

            if (gamepad1.a) {
                claw.move();
            }


            telemetry.addData("Power", power);
            telemetry.addData("Strafe", strafe);
            telemetry.addData("Turn", turn);
            telemetry.update();


        }
    }
}