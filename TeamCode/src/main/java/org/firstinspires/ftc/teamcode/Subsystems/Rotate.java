//package org.firstinspires.ftc.teamcode.Subsystems;
//
//import com.acmerobotics.dashboard.config.Config;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.hardware.Servo;
//
//@Config
//public class Rotate {
//    public static double OPEN_POSITION = 0.85;
//    public static double CLOSE_POSITION = 0.25;
//    public boolean open;
//    public Servo rotate;
//
//    public void init(HardwareMap map) {
//        rotate = map.get(Servo.class, "rotate");
//        open = false;
//        rotate.setPosition(CLOSE_POSITION);
//    }
//
//    public void move(boolean a, boolean b) {
//        if(a) {
//            rotate.setPosition(OPEN_POSITION);
//        } else if (b){
//            rotate.setPosition(CLOSE_POSITION);
//        }
//    }
//
//    public void open() {
//        rotate.setPosition(OPEN_POSITION);
//    }
//
//    public void close() {
//        rotate.setPosition(CLOSE_POSITION);
//    }
//
//    public double getPosition() {
//        return rotate.getPosition();
//    }
//
//
//}