package org.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Lift {
    public DcMotor lift;

    public void init(HardwareMap map) {
        lift = map.dcMotor.get("lift");
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void move(boolean up, boolean down) {
        if (up){
            lift.setPower(-0.3);
        } else if (down) {
            lift.setPower(-0.3);
        } else{
            lift.setPower(0.0);
        }
    }

}