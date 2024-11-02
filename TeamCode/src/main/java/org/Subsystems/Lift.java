package org.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Lift {
    public DcMotor lift;

    public void init(HardwareMap map) {
        lift = map.dcMotor.get("lift");
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void move(boolean up, boolean down, double ff, int position) {
        if (up && position < 440){
            lift.setPower(-0.2);
        } else if (down) {
            lift.setPower(0.2);
        } else {
            lift.setPower(-ff);
        }
    }

    public int position() {
        return lift.getCurrentPosition();
    }

}