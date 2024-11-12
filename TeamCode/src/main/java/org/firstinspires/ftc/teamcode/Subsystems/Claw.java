package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Claw {
    private static final double OPEN_POSITION = 0.5;
    private static final double CLOSE_POSITION = 0.1;
    public boolean open;
    public Servo claw;

    public void init(HardwareMap map) {
        claw = map.get(Servo.class, "claw");
        open = false;
        claw.setPosition(0.1);
    }

    public void move(boolean a, boolean b) {
        if(a) {
            this.open();
        } else if (b){
            this.close();
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