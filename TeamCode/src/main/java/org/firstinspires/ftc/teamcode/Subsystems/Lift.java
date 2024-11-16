package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;


@Config
@Autonomous
public class Lift extends SubsystemBase {
    public DcMotor lift;

    private PIDController controller;
    public LiftStates liftStates = LiftStates.DOWN;
    public static double p = 0.003, i = 0, d = 0;
    public static double f = 0;
    public static int target = 0;
    private final double ticks_in_degree = 1425.1 / 360.0;
    public static boolean isUp = false;


    public static int DOWN = -20, AUTON_POS_LOW = -350, AUTON_POS = -460, POS1_POS = 0, POS2_POS = 0, POS3_POS = 0;

    public enum LiftStates {
        DOWN,
        AUTON,
        AUTON_POS_LOW,
        POS1,
        POS2,
        POS3,
    }

    //FIX THIS!!!!


    public void init(HardwareMap map) {
        lift = map.dcMotor.get("lift");
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        controller = new PIDController(p, i, d);
//        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    public void update(LiftStates state) {
        isUp = state != LiftStates.DOWN;
        switch (state) {
            case DOWN:
                target = DOWN;
                liftStates = state;
                break;
            case AUTON:
                target = AUTON_POS;
                liftStates = state;
                break;
            case POS1:
                target = POS1_POS;
                liftStates = state;
                break;
            case POS2:
                target = POS2_POS;
                liftStates = state;
                break;
            case POS3:
                target = POS3_POS;
                liftStates = state;
                break;
            case AUTON_POS_LOW:
                target = AUTON_POS_LOW;
                liftStates = state;
        }
    }

    public int loop() {
        controller.setPID(p, i, d);
        int pos = lift.getCurrentPosition();
        double pid = controller.calculate(pos, target);
        double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;
        double power = (pid + ff)/2.5;
        if (target == DOWN) {
            lift.setPower(0);
            lift.setPower(0);
        }
        else {
            lift.setPower(power);
            lift.setPower(power);
        }
        return (target - pos);
//        telemetry.addData("pos", pos);
//        telemetry.addData("power", power);
//        telemetry.update();
    }

    public void move(boolean up, boolean down, double ff, int position) {
        if (up && position < 290){
            lift.setPower(-0.2);
        } else if (down) {
            lift.setPower(0.2);
        } else {
            lift.setPower(-ff);
        }
    }

    public void down() {
        lift.setPower(0.7);
    }

    public void up(){
        lift.setPower(-0.1);
    }

    public int position() {
        return lift.getCurrentPosition();
    }

    public void setPower(double power){
        lift.setPower(power);
    }


}