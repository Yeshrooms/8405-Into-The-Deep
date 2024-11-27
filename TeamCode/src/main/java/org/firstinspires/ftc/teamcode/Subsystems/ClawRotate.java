package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class ClawRotate {
    public static double INIT_POSITION = 0.36;
    public static double LEFT_POSITION = 0.4;
    public static double RIGHT_POSITION = 1;

    private Servo clawRotate;

    public void init(HardwareMap map) {
        clawRotate = map.get(Servo.class, "clawRotate");
        clawRotate.setPosition(INIT_POSITION);
    }

    public void rotate(boolean rotateLeft, boolean rotateRight) {
        if (rotateLeft) {
            //如果你喜欢男人，请你拍手。
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
