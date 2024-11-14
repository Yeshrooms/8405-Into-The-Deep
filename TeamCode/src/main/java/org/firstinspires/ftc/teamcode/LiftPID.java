package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Subsystems.Lift;


@Config
@TeleOp
public class LiftPID extends LinearOpMode{

    private PIDController controller;
    public static double p = 0.003, i = 0, d = 0.0;
    public static double f = 0.0;

    private Lift lift = new Lift();

    public static int target = 0;

    private final double ticks_in_degree = 1425.1/360.0;




    public void setTarget(int targetPos) {
        target = targetPos;
    }


    @Override
    public void runOpMode() throws InterruptedException {

        controller = new PIDController(p, i, d);
        lift.init(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        while (opModeIsActive()) {
            controller.setPID(p, i, d);
            int pos = lift.position();
            double pid = controller.calculate(pos, target);
            double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;

            double power = pid + ff;

            lift.setPower(power);
            telemetry.addData("pos", pos);
            telemetry.addData("power", power);
            telemetry.update();
        }
    }
}