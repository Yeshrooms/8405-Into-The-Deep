package org.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LiftExtendo {
    public DcMotor lift;
    public DcMotor extendo;

    public void init(HardwareMap map) {
        lift = map.dcMotor.get("lift");
        extendo = map.dcMotor.get("extendo");
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extendo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        extendo.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void up() {
        lift.setTargetPosition(-250);
        lift.setPower(-0.2);
        extendo.setTargetPosition(200);
        extendo.setPower(0.2);
    }

    public void down() {
        lift.setTargetPosition(250);
        lift.setPower(0.2);
        extendo.setTargetPosition(200);
        extendo.setPower(0.2);
    }

    public void reset() {
        lift.setTargetPosition(0);
        extendo.setTargetPosition(0);
        lift.setPower(0.2);
        extendo.setPower(0.2);
    }
}

/*package org.Subsystems;

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