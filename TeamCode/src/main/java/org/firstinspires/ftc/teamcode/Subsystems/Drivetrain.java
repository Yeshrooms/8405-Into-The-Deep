package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@Config

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

    private double[] statesArray = {AWAY_BAR_1, -SIXTY};

    public static double norm = 1.4;
    public static double rightNorm = 1.3;

    public Pose2d[] points = {
            new Pose2d(0.0,10.0,Rotation2d.fromDegrees(0)),
            new Pose2d(0.0,11.0,Rotation2d.fromDegrees(0)),
            new Pose2d(0.0,12.0,Rotation2d.fromDegrees(0)),
            new Pose2d(0.0, TO_BAR_1, Rotation2d.fromDegrees(0)),
//            new Pose2d(0.0, 15.0, Rotation2d.fromDegrees(0)),
//            new Pose2d(0.0, 15.0, Rotation2d.fromDegrees(-90)),
    };

    private GoBildaPinpointDriver odo;
    public double target = 0.0;

    public static double targetAngle = 0;

    public static double TO_BAR_1 = 20.5, AWAY_BAR_1 = 7, RIGHT_BLOCK_1 = 0, UP_BLOCK = 0, RIGHT_BLOCK_2 = 0, PUSH_BLOCK = 0, ZERO = 0, THIRTY = 30, SIXTY = 60;

    public Drivetrain.DriveStates driveStates = DriveStates.TO_BAR_1;


    public enum DriveStates {
        TO_BAR_1,
        AWAY_BAR_1,
        RIGHT_BLOCK_1,
        UP_BLOCK,
        RIGHT_BLOCK_2,
        PUSH_BLOCK,
        ZERO,
        THIRTY,
        SIXTY,
    }

    public void update(Drivetrain.DriveStates state) {
        switch (state) {
            case TO_BAR_1:
                target = TO_BAR_1;
                driveStates = state;
                break;
            case AWAY_BAR_1:
                target = AWAY_BAR_1;
                driveStates = state;
                break;
//            case POS1:
//                target = POS1_POS;
//                liftStates = state;
//                break;
//            case POS2:
//                target = POS2_POS;
//                liftStates = state;
//                break;
//            case POS3:
//                target = POS3_POS;
//                liftStates = state;
//                break;
//            case AUTON_POS_LOW:
//                target = AUTON_POS_LOW;
//                liftStates = state;
            case ZERO:
                targetAngle = ZERO;
                driveStates = state;
                break;
            case SIXTY:
                targetAngle = SIXTY;
                driveStates = state;
                break;
            case THIRTY:
                targetAngle = THIRTY;
                driveStates = state;
                break;
        }
    }

    public void update(int index) {
        if (index == 1){
            targetAngle = statesArray[index];

        } else {
            target = statesArray[index];
        }
    }

    private PIDController yLinearController;
    private PIDController angularController;

    public static double lp= 0.037, li = 0.0, ld = 0.0; //0.024
    public static double ap= 0.048, ai = 0.0, ad = 0.0;

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


        odo = map.get(GoBildaPinpointDriver.class,"odo");

        // configure
        odo.setOffsets(-84.0, -168.0);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.resetPosAndIMU();

        yLinearController = new PIDController(lp, li, ld);

        angularController = new PIDController(ap, ai, ad);
    }

    public boolean loop(Telemetry telemetry){
//        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        odo.update();
        Pose2d targetPose = points[(int)target];
        double yTarget = targetPose.getY();
        double targetAngle = Math.toDegrees(targetPose.getHeading());
        Pose2D currentPosition = odo.getPosition();

        double currentDistance = -currentPosition.getY(DistanceUnit.INCH);
        double currentHeading = currentPosition.getHeading(AngleUnit.DEGREES);

        // distance
        double distanceError = yTarget - currentDistance;
        double forwardPower = yLinearController.calculate(currentDistance, yTarget);
        if (forwardPower > 0.6){
            forwardPower = 0.6;
        }
        // angle
        double angleError = targetAngle - currentHeading;
        double anglePower = angularController.calculate(currentHeading, targetAngle);

        // maths more like meths
        double leftPower = forwardPower - anglePower;
        double rightPower = forwardPower + anglePower;

        fL.setPower(-leftPower*norm);
        bL.setPower(leftPower*norm);
        fR.setPower(rightPower*norm*rightNorm);
        bR.setPower(-rightPower*norm);
        telemetry.addData("forward power" , forwardPower);
        telemetry.addData("left power" , leftPower);
        telemetry.addData("right power" , rightPower);
        telemetry.addData("disntance reorrr", distanceError);
        telemetry.addData("angle error", angleError);

        return distanceError < 1 && angleError < 5;

//        // distance
//
//        double forwardPower = -yLinearController.calculate(yDistance, yTarget);
//        // angle
//        double anglePower = angularController.calculate(Math.toDegrees(currentHeading), targetAngle);

//
//        // Rotate the movement direction counter to the bot's rotation
//        double rotX = forwardPower * Math.cos(currentHeading) - strafePower * Math.sin(currentHeading);
//        double rotY = forwardPower * Math.sin(currentHeading) + strafePower * Math.cos(currentHeading);
//
//        rotX = rotX * 1.1;  // Counteract imperfect strafing
//
//        // Denominator is the largest motor power (absolute value) or 1
//        // This ensures all the powers maintain the same ratio,
//        // but only if at least one is out of the range [-1, 1]
//        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(anglePower), 1);
//        double frontLeftPower = (rotY + rotX + anglePower) / denominator;
//        double backLeftPower = (rotY - rotX + anglePower) / denominator;
//        double frontRightPower = (rotY - rotX - anglePower) / denominator;
//        double backRightPower = (rotY + rotX - anglePower) / denominator;
//
//        fL.setPower(frontLeftPower);
//        bL.setPower(backLeftPower);
//        fR.setPower(frontRightPower);
//        bR.setPower(backRightPower);
//
//        boolean atX = Math.abs(xTarget - xDistance) < 1.5;
//        boolean atY = Math.abs(yTarget - yDistance) < 1.5;
//        boolean atHeading = Math.abs(targetAngle - currentHeading) < 5;
//
//        telemetry.addData("xtar", xTarget);
//        telemetry.addData("ytar", yTarget);
//        telemetry.addData("headingtar", targetAngle);
//        telemetry.addData("x", xDistance);
//        telemetry.addData("y", yDistance);
//        telemetry.addData("heading", currentHeading);
//        telemetry.addData("forPow", forwardPower);
//        telemetry.addData("strPow", strafePower);
//        telemetry.addData("angPow", anglePower);
//        telemetry.addData("atXerror", atX);
//        telemetry.addData("atYerror", atY);
//        telemetry.addData("atHeadingError", atHeading);
//        return atX && atY && atHeading;
    }

//    public boolean loop(Telemetry telemetry){
////        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//        odo.update();
//        Pose2d targetPose = points[target];
//        double xTarget = targetPose.getX();
//        double yTarget = targetPose.getY();
//        double targetAngle = Math.toDegrees(targetPose.getHeading());
//        Pose2D currentPosition = odo.getPosition();
//        double yDistance = -currentPosition.getY(DistanceUnit.INCH);
//        double xDistance = -currentPosition.getX(DistanceUnit.INCH);
//        double currentHeading = currentPosition.getHeading(AngleUnit.RADIANS);
//
//        // distance
//
//        double forwardPower = -yLinearController.calculate(yDistance, yTarget);
//        double strafePower = -xLinearController.calculate(xDistance, xTarget);
//
//        // angle
//        double anglePower = angularController.calculate(Math.toDegrees(currentHeading), targetAngle);
//
//
//        // Rotate the movement direction counter to the bot's rotation
//        double rotX = forwardPower * Math.cos(currentHeading) - strafePower * Math.sin(currentHeading);
//        double rotY = forwardPower * Math.sin(currentHeading) + strafePower * Math.cos(currentHeading);
//
//        rotX = rotX * 1.1;  // Counteract imperfect strafing
//
//        // Denominator is the largest motor power (absolute value) or 1
//        // This ensures all the powers maintain the same ratio,
//        // but only if at least one is out of the range [-1, 1]
//        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(anglePower), 1);
//        double frontLeftPower = (rotY + rotX + anglePower) / denominator;
//        double backLeftPower = (rotY - rotX + anglePower) / denominator;
//        double frontRightPower = (rotY - rotX - anglePower) / denominator;
//        double backRightPower = (rotY + rotX - anglePower) / denominator;
//
//        fL.setPower(frontLeftPower);
//        bL.setPower(backLeftPower);
//        fR.setPower(frontRightPower);
//        bR.setPower(backRightPower);
//
//        boolean atX = Math.abs(xTarget - xDistance) < 4;
//        boolean atY = Math.abs(yTarget - yDistance) < 8;
//        boolean atHeading = Math.abs(targetAngle - currentHeading) < 10;
//
//        telemetry.addData("xtar", xTarget);
//        telemetry.addData("ytar", yTarget);
//        telemetry.addData("headingtar", targetAngle);
//        telemetry.addData("x", xDistance);
//        telemetry.addData("y", yDistance);
//        telemetry.addData("heading", currentHeading);
//        telemetry.addData("forPow", forwardPower);
//        telemetry.addData("strPow", strafePower);
//        telemetry.addData("angPow", anglePower);
//        telemetry.addData("atXerror", atX);
//        telemetry.addData("atYerror", atY);
//        telemetry.addData("atHeadingError", atHeading);
//        return atX && atY && atHeading;
//    }


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