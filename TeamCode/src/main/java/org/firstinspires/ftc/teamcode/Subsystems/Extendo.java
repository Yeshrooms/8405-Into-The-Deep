package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Extendo {
    public DcMotor extendo;

    public void init(HardwareMap map) {
        extendo = map.dcMotor.get("extendo");
        extendo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void extend(float out, float in) {
        if (out > 0){
            extendo.setPower(-0.15);
        } else if (in > 0) {
            extendo.setPower(0.1);
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