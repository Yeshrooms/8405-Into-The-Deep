package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Claw {
    public static double OPEN_POSITION = 0.65;
    public static double CLOSE_POSITION = 0.42;
    public boolean open;
    public Servo claw;

    public void init(HardwareMap map) {
        claw = map.get(Servo.class, "claw");
        open = false;
        claw.setPosition(CLOSE_POSITION);
    }

    public void move(boolean a, boolean b) {
        if(a) {
            claw.setPosition(OPEN_POSITION);
        } else if (b){
            claw.setPosition(CLOSE_POSITION);
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