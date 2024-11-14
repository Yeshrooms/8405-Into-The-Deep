package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Drivetrain {
    public DcMotor fL;
    public DcMotor bL;
    public DcMotor fR;
    public DcMotor bR;
    //    public Encoder encoder;
//    public Encoder encoder2;
    //public static double TICKS_PER_REV = 1;
    public static double GEAR_RATIO = 1;
    public static double WHEEL_RADIUS_INCHES = 1.88976;
    public static double TICKS_PER_REV = 537.6;
    public IMU imu;

    public static double p = 0.0, i = 0.0, d = 0.0;
    private PIDController controller;

    private GoBildaPinpointDriver odo;

    public static int TO_BAR_1 = 0, AWAY_BAR_1 = 0, RIGHT_BLOCK_1 = 0, UP_BLOCK = 0, RIGHT_BLOCK_2 = 0, PUSH_BLOCK = 0;


    public enum DriveStates {
        TO_BAR_1,
        AWAY_BAR_1,
        RIGHT_BLOCK_1,
        UP_BLOCK,
        RIGHT_BLOCK_2,
        PUSH_BLOCK,
    }

    public static int ZERO = 0, THIRTY = 30, SIXTY = 60, NINETY = 90, FOURTY_FIVE = 45, TWENTY = 20;


    public enum AngleStates {
        ZERO,
        THIRTY,
        SIXTY,
        NINETY,
        FOURTY_FIVE,
        TWENTY,
    }

    public void init(HardwareMap map) {
        fL = map.dcMotor.get("leftFront");
        bL = map.dcMotor.get("leftBack");
        fR = map.dcMotor.get("rightFront");
        bR = map.dcMotor.get("rightBack");
//        encoder = new Encoder(map.get(DcMotorEx.class, "frontLeft"));
//        encoder2 = new Encoder(map.get(DcMotorEx.class, "backRight"));
//        imu = map.get(IMU.class, "imu");

        fL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Reset the motor encoder
        bR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Reset the motor encoder
        fR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);// Reset the motor encoder

        fL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        fL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        bL.setDirection(DcMotorSimple.Direction.REVERSE);
        bR.setDirection(DcMotorSimple.Direction.REVERSE);

        controller = new PIDController(p, i ,d);

        odo = map.get(GoBildaPinpointDriver.class,"odo");

        // configure
        odo.setOffsets(-84.0, -168.0);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.resetPosAndIMU();

    }

//    public int loop(){
//        controller.setPID(p, i, d);
//        //            odo.update();
////            Pose2D currentPosition = odo.getPosition();
////            double currentDistance = currentPosition.getX(DistanceUnit.MM);
////            double currentHeading = currentPosition.getHeading(AngleUnit.DEGREES);
////
////            // distance
////            double distanceError = targetDistance - currentDistance;
////            double forwardPower = linearController.calculate(distanceError);
////
////            // angle
////            double angleError = targetAngle - currentHeading;
////            double anglePower = angularController.calculate(angleError);
////
////            // maths more like meths
////            double leftPower = forwardPower - anglePower;
////            double rightPower = forwardPower + anglePower;
////            drive.setPowers(leftPower, leftPower, rightPower, rightPower);
//        int pos = .position();
//        double pid = controller.calculate(pos, target);
//        double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;
//
//        double power = pid + ff;
//
//        lift.setPower(power);
//        telemetry.addData("pos", pos);
//        telemetry.addData("power", power);
//        telemetry.update();
//    }

    public double getHeading(){
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    public int getPosition(){
        return bL.getCurrentPosition();
    }

//    public double get1Position() {
//        return -encoder.getCurrentPosition();
//    }
//
//    public double get2Position() {
//        return -encoder2.getCurrentPosition();
//    }

    public void moveMoveMOVE(double power){
        double fLPow = power;
        double bLPow = power;
        double fRPow = power;
        double bRPow = power;
        setPowers(fLPow,bLPow,fRPow,bRPow);
    }
    //o
    // left joystick controls forward/backward and strafe, right controls turning
    public void move(double power, double strafe, double turn) {
        // normalize so doesn't exceed 1
        //double norm = Math.max(Math.abs(power) + Math.abs(strafe) + Math.abs(turn), 1);
        double norm = 1;
        double fLPow = power + strafe + turn;
        double bLPow = power - strafe + turn;
        double fRPow = power - strafe - turn;
        double bRPOw = power + strafe - turn;

        setPowers(-fLPow/norm, bLPow/norm, fRPow/norm, -bRPOw/norm);
    }

    public void setPowers(double fLPow, double bLPow, double fRPow, double bRPOw) {
        fL.setPower(fLPow);
        bL.setPower(bLPow);
        fR.setPower(fRPow);
        bR.setPower(bRPOw);
    }

    public void setPowers(double pow) {
        this.setPowers(pow, pow, pow, pow);
    }

    public void setTurnPower(double pow) {
        fL.setPower(pow);
        bL.setPower(pow);
        fR.setPower(-pow);
        bR.setPower(-pow);
    }
}