//package org.firstinspires.ftc.teamcode.Subsystems;
//
//import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.arcrobotics.ftclib.controller.PIDController;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//
//import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
//
//
//@Config
//@Autonomous
//public class Lift extends LinearOpMode {
//    public DcMotor lift;
//
//    private PIDController controller;
//    public static double p = 0, i = 0, d = 0;
//    public static double f = 0;
//    public static int target = 0;
//    private final double ticks_in_degree = 700 / 180.0;
//    //FIX THIS!!!!
//
//    public void runOpMode(HardwareMap map) {
//        lift = map.dcMotor.get("lift");
//        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//        controller = new PIDController(p, i, d);
//        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//    }
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//
//    }
//
//    public void loop(){
//        controller.setPID(p, i, d);
//        int armPos = lift.getCurrentPosition();
//        double pid = controller.calculate(armPos, target);
//        double ff = Math.cos (Math. toRadians(target / ticks_in_degree)) * f;
//        double power = pid + ff;
//        lift.setPower (power);
//        telemetry.addData("pos", armPos);
//    }
//
////    public void move(boolean up, boolean down){
////
////    }
//
//    public void move(boolean up, boolean down, double ff, int position) {
//        if (up && position < 290){
//            lift.setPower(-0.2);
//        } else if (down) {
//            lift.setPower(0.2);
//        } else {
//            lift.setPower(-ff);
//        }
//    }
//
//    public void down() {
//        lift.setPower(0.7);
//    }
//
//    public void up(){
//        lift.setPower(-0.1);
//    }
//
//    public int position() {
//        return lift.getCurrentPosition();
//    }
//
//}