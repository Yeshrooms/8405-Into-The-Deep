package org.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Extendo {
    public DcMotor extendo;

    public void init(HardwareMap map) {
        extendo = map.dcMotor.get("extendo");
        extendo.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void extend(float out, float in) {
        if (out > 0){
            extendo.setPower(-0.3);
        } else if (in > 0) {
            extendo.setPower(0.3);
        } else{
            extendo.setPower(0.0);
        }
    }

}

/*
 * // Set the motor to RUN_TO_POSITION mode
motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

// Set the target position
motor.setTargetPosition(targetPosition);

// Set the power to start moving
motor.setPower(1.0); // Set to maximum power

// Wait until the motor reaches the target position
while (motor.isBusy()) {
    // Optional: Monitor current position or other logic
}

// Stop the motor
motor.setPower(0.0);

 */