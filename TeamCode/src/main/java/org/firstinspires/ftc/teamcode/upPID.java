package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Subsystems.ArmGroup;
import org.firstinspires.ftc.teamcode.Subsystems.Extendo;
import org.firstinspires.ftc.teamcode.Subsystems.Lift;


@Config
@TeleOp
public class upPID extends LinearOpMode{

    private PIDController controller;
    public static double p = 0.01, i = 0.01, d = 0.0;
    public static double f = 0.0;

    private Extendo extendo = new Extendo();

    public static int target = 1500;

    private final double ticks_in_degree = 103.8/360.0;


    public void setTarget(int targetPos) {
        target = targetPos;
    }


    @Override
    public void runOpMode() throws InterruptedException {

        controller = new PIDController(p, i, d);
        extendo.init(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();


        while (opModeIsActive()) {
            int pos = extendo.getPosition();
//            extendo.extend(gamepad1.right_trigger, gamepad1.left_trigger);
            controller.setPID(p, i, d);
            double pid = controller.calculate(pos, target);
            double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;

            double power = pid + ff;

            extendo.extend((float) power);
            telemetry.addData("power", power);
            telemetry.addData("position", pos);
            telemetry.update();
        }
    }
}