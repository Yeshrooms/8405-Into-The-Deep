package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ClawRotate {
    private static final double LEFT_POSITION = 0.3;
    private static final double RIGHT_POSITION = 0.7;

    private Servo clawRotate;

    public void init(HardwareMap map) {
        clawRotate = map.get(Servo.class, "clawRotate");
        clawRotate.setPosition(LEFT_POSITION);
    }

    public void rotate(boolean rotateLeft, boolean rotateRight) {
        if (rotateLeft) {
            rotateLeft();
        } else if (rotateRight) {
            rotateRight();
        }
    }

    public void rotateLeft() {
        clawRotate.setPosition(LEFT_POSITION);
    }

    public void rotateRight() {
        clawRotate.setPosition(RIGHT_POSITION);
    }

    public double getPosition() {
        return clawRotate.getPosition();
    }
}
