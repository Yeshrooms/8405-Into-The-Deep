package org.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Extendo {
    public DcMotor extendo;

    public void init(HardwareMap map) {
        extendo = map.dcMotor.get("extendo");
        extendo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void extend(float out, float in) {
        if (out > 0){
            extendo.setPower(-0.4);
        } else if (in > 0) {
            extendo.setPower(0.4);
        } else{
            extendo.setPower(0.0);
        }
    }
}