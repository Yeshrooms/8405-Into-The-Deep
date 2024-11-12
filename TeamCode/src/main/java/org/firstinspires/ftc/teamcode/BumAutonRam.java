//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
//import org.firstinspires.ftc.teamcode.Subsystems.Claw;
//import org.firstinspires.ftc.teamcode.Subsystems.Lift;
//import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
//
//@Autonomous
//public class BumAutonRam extends LinearOpMode {
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//
//
//        Drivetrain drive = new Drivetrain();
//        Lift lift = new Lift();
//        Claw claw = new Claw();
//        drive.init(hardwareMap);
//        lift.init(hardwareMap);
//        claw.init(hardwareMap);
//        double norm = 0.65;
//
//        waitForStart();
//        drive.move(0.5*norm, 0.0, 0.0);
//        sleep(3000);
//        drive.move(0.0,0.0,0.0);
//        sleep(500);
//        claw.close();
//        drive.move(-0.1, 0.0,0.0);
//        sleep(300);
//        lift.down();
//        //drive.move(-0.3*norm, 0.0, -0.09*norm);
//        sleep(1000);
//        claw.open();
//        drive.move(-0.2, 0.0,0.0);
//
//
//
//    }
//}
