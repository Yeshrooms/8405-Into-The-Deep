package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class ArmGroup {
//    public static double OPEN_POSITION = 0.95;
//    public static double CLOSE_POSITION = 0.65;
    public static double INIT_POSITION = 0.2;
    public static double SCORE = 0.48;
    public static double INTO_SUB = 0.75;
    public static double IN_SUB = 0.78;
    public static double OUT_SUB = 0.75;
    public static double OBSERVATION = 0.772;

    double[] subs = {0.84, 1, 0.93};
    int subcount = 0;

    private boolean goIn = true;

    private double into = 0.84;
    private double in = 1;
    private double out = 0.93;

    public double next;
    public Servo left;
    public Servo right;

    public void init(HardwareMap map) {
        left = map.get(Servo.class, "leftArm");
        right = map.get(Servo.class, "rightArm");

        next = into;
        left.setPosition(INIT_POSITION);
        right.setPosition(1-INIT_POSITION);
    }

    public void move(boolean rightbump, boolean leftbump, boolean y, boolean b) {
        if(rightbump) {
            left.setPosition(SCORE);
            right.setPosition(1-SCORE);
        } else if (leftbump){
            left.setPosition(OBSERVATION);
            right.setPosition(1-OBSERVATION);
        } else if (y){

            left.setPosition(INTO_SUB);
            right.setPosition(1-INTO_SUB);

//            left.setPosition(next);
//            left.setPosition(1-next);
//            subcount++;
//            subcount%=3;
//            next = subs[subcount];
        }
        else if (b){
            left.setPosition(OBSERVATION);
            right.setPosition(1-OBSERVATION);
        }
    }

    public void initpos() {
        left.setPosition(INIT_POSITION);
        right.setPosition(1-INIT_POSITION);
    }
//
//    public void open() {
//        left.setPosition(OPEN_POSITION);
//        right.setPosition(1-    OPEN_POSITION);
//    }

//    public void close() {
//        rotate.setPosition(CLOSE_POSITION);
//    }

    public double getPosition() {
        return left.getPosition();
    }


}