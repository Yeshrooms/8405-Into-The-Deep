package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LiftPID {
    public DcMotorEx lift;

    private PIDController controller;
    public static double p = 0.01, i = 0, d = 0.001;
    public static double f = 0.01;

    public static int target = 0;

    private final double ticks_in_degree = 8192/360.0;

    public LiftPID(HardwareMap hardwareMap) {
        controller = new PIDController(p, i, d);
        lift = hardwareMap.get(DcMotorEx.class, "lift");
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void run() {
        controller.setPID(p, i, d);
        int pos = lift.getCurrentPosition();
        double pid = controller.calculate(pos, target);
        double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;

        double power = pid + ff;

        lift.setPower(power);
    }

    public void setTarget(int targetPos) {
        target = targetPos;
    }
}