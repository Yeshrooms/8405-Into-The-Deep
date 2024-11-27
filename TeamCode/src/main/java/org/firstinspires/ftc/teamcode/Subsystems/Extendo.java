package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config

public class Extendo {
    public DcMotor extendo;

    public static double dick = 0.3;
    public static double bigDick = 0.4;

    public void init(HardwareMap map) {
        extendo = map.dcMotor.get("extendo");
        extendo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void extend(float out, float in) {
        if (out > 0){
            extendo.setPower(-out*dick);
        } else if (in > 0) {
            extendo.setPower(in*bigDick);
        } else{
            extendo.setPower(0.0);
        }
    }

    public void retract(){
        extendo.setPower(0.4);
    }


    public void out(){
        extendo.setPower(-0.3);
    }
}