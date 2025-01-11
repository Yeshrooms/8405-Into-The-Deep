package org.firstinspires.ftc.teamcode.Subsystems;

import static java.lang.Math.abs;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config

public class Extendo {
    public DcMotor extendo;
    public DcMotor extendoTwo;


    public Extendo.ExtendoStates extendoStates = Extendo.ExtendoStates.ZERO;


    public static double p = 0.01, i = 0.01, d = 0.0;
    public static double f = 0.0;
    public static int target = 0;

    private final double ticks_in_degree = 103.8/360.0;

    private PIDController controller = new PIDController(p, i, d);

    public static int SPECIMEN = 550, BASKET = 1780, ZERO = 0, AUTON = 650;

    public enum ExtendoStates {
        SPECIMEN,
        BASKET,
        ZERO,
        AUTON
    }

    public void update(Extendo.ExtendoStates state) {
        switch (state) {
            case SPECIMEN:
                target = SPECIMEN;
                extendoStates = state;
                break;
            case BASKET:
                target = BASKET;
                extendoStates = state;
                break;
            case ZERO:
                target = ZERO;
                extendoStates = state;
                break;
            case AUTON:
                target = AUTON;
                extendoStates = state;
                break;
        }
    }

    public void init(HardwareMap map) {
        extendo = map.dcMotor.get("extendo");
        extendoTwo = map.dcMotor.get("extendono");
        extendo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extendo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        extendoTwo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extendoTwo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

//    public void extend(float out, float in) {
//        if (out > 0){
//            extendo.setPower(-out*dick);
//        } else if (in > 0) {
//            extendo.setPower(in*bigDick);
//        } else{
//            extendo.setPower(0.0);
//        }
//    }


    public void extend(float go){
        extendo.setPower(-go);
        extendoTwo.setPower(go);
    }

    public boolean loop() {
        controller.setPID(p, i, d);
        int pos = -extendo.getCurrentPosition();
        double pid = controller.calculate(pos, target);
        double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;
        double power = pid + ff;
        extendo.setPower(-power);
        extendoTwo.setPower(power);
        return abs(pos-target) < 7;
//        telemetry.addData("lift pos", pos);
//        telemetry.addData("lift power", power);
//        telemetry.addData("lift error", target-pos);
//        telemetry.update();
    }

    public int getPosition(){
        return -extendo.getCurrentPosition();
    }

    public void retract(){
        extendo.setPower(0.4);
    }


    public void out(){
        extendo.setPower(-0.3);
    }
}