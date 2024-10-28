package org.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Claw {
    private static final double OPEN_POSITION = 0.5;
    private static final double CLOSE_POSITION = 0.0;
    public boolean open = false;
    public Servo claw;

    public void init(HardwareMap map) {
        claw = map.servo.get("claw");
        claw.setPosition(0.0); // daniel gay!
    }

    public void move() {
        if(open) {
            open = false;
            this.close();
        } else {
            open = true;
            this.open();
        }
    }

    public void open() {
        claw.setPosition(OPEN_POSITION);
    }

    public void close() {
        claw.setPosition(CLOSE_POSITION);
    }
    
    public double getPosition() {
        return claw.getPosition();
    }


}