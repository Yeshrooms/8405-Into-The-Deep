//package org.firstinspires.ftc.teamcode;
//
//import com.acmerobotics.roadrunner.Pose2d;
//import com.qualcomm.hardware.lynx.LynxModule;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import org.firstinspires.ftc.teamcode.Subsystems.Claw;
//import org.firstinspires.ftc.teamcode.Subsystems.ClawRotate;
//import org.firstinspires.ftc.teamcode.Subsystems.Extendo;
//import org.firstinspires.ftc.teamcode.Subsystems.Lift;
//import java.util.List;
//
//@TeleOp
//public class unorzTeleOp extends LinearOpMode {
//
//    private static boolean active = true;
//    private Lift lift = new Lift();
//    private Extendo extendo = new Extendo();
//    private Claw claw = new Claw();
//    private ClawRotate clawRotate = new ClawRotate();
//
//    @Override
//    public void runOpMode() {
//        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
//        for (LynxModule hub : allHubs) {
//            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
//        }
//
//
//        DcMotorEx backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
//        DcMotorEx backRight = hardwareMap.get(DcMotorEx.class, "backRight");
//        DcMotorEx frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
//        DcMotorEx frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
//
//        Pose2d start = new Pose2d(0.0, 0.0, 0.0);
//        SparkFunOTOSDrive drive = new SparkFunOTOSDrive(hardwareMap, start);
//
//        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
//        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
//
//        backLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//        backRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//        frontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//        frontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//
//        double lastTime = 0.0;
//
//        lift.init(hardwareMap);
//        extendo.init(hardwareMap);
//        claw.init(hardwareMap);
//        clawRotate.init(hardwareMap);
//
//        waitForStart();
//
//        while (opModeIsActive()) {
//            drive.updatePoseEstimate();
//
//            double y = -gamepad1.left_stick_y;
//            double x = -gamepad1.left_stick_x;
//            double r = -gamepad1.right_stick_x;
//
////            double heading = -drive.getPose().getRotation().getRadians();
//
//            // adjust based on orientation
//            double rotX = x * Math.cos(-heading) - y * Math.sin(-heading);
//            double rotY = x * Math.sin(-heading) + y * Math.cos(-heading);
//
//            double den = Math.max(Math.abs(rotX) + Math.abs(rotY) + Math.abs(r), 1.0);
//            double norm = 1.0;
//
//            if (active) {
//                frontLeft.setPower((rotY + rotX + r) / den * norm);
//                backLeft.setPower((rotY - rotX + r) / den * norm);
//                frontRight.setPower((rotY - rotX - r) / den * norm);
//                backRight.setPower((rotY + rotX - r) / den * norm);
//            } else {
//                frontLeft.setPower(0.0);
//                backLeft.setPower(0.0);
//                frontRight.setPower(0.0);
//                backRight.setPower(0.0);
//            }
//
//            if (gamepad1.right_bumper) {
//                lift.update(Lift.LiftStates.AUTON);
//            } else if (gamepad1.left_bumper) {
//                lift.update(Lift.LiftStates.AUTON_POS_LOW);
//            } else if (gamepad1.dpad_up) {
//                lift.update(Lift.LiftStates.DOWN);
//            }
//
//
//            extendo.extend(gamepad1.right_trigger, gamepad1.left_trigger);
//
//
//            claw.move(gamepad1.a, gamepad1.x);
//
//            if (gamepad1.b) {
//                clawRotate.rotate(true, false); // Rotate left
//            } else if (gamepad1.y) {
//                clawRotate.rotate(false, true); // Rotate right
//            }
//
//            telemetry.addData("Claw position", claw.getPosition());
//            telemetry.addData("Lift position", lift.position());
//            telemetry.addData("ClawRotate position", clawRotate.getPosition());
//            telemetry.addData("Power", -gamepad1.left_stick_y);
//            telemetry.addData("Strafe", gamepad1.left_stick_x);
//            telemetry.addData("Turn", gamepad1.right_stick_x);
//            telemetry.update();
//
//            lastTime = System.currentTimeMillis();
//
//            for (LynxModule hub : allHubs) {
//                hub.clearBulkCache();
//            }
//        }
//    }
//}
