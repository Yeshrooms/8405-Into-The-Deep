package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Subsystems.Extendo;
import org.firstinspires.ftc.teamcode.Subsystems.Lift;


@Config
@TeleOp
public class extendoPID extends LinearOpMode{

    private PIDController controller;
    public static double p = 0.0, i = 0, d = 0.0;
    public static double f = 0.0;

    private Extendo extendo = new Extendo();

    public static int target = 0;

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
//
//        while (opModeIsActive()) {
//            controller.setPID(p, i, d);
//            double pid = controller.calculate(pos, target);
//            double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;
//
//            double power = pid + ff;
//
//            lift.setPower(power);
//            telemetry.addData("power", power);
//            telemetry.update();
//        }
    }
}