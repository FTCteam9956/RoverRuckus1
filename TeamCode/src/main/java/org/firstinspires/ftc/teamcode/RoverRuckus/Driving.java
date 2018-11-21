//package org.firstinspires.ftc.teamcode.RoverRuckus;
//
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//
//@TeleOp(name = "Driving", group = "Teleop")
////@Disabled
//public class Driving extends LinearOpMode {
//
//    public DcMotor left1;
//    public DcMotor right1;
//
//    public void runOpMode(){
//
//        left1 = hardwareMap.dcMotor.get("left1");
//        right1 = hardwareMap.dcMotor.get("right1");
//
//        left1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        right1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//        right1.setDirection(DcMotorSimple.Direction.REVERSE);
//        waitForStart();
//
//        while(opModeIsActive()){
//            left1.setPower(gamepad1.left_stick_y *.5);
//            right1.setPower(gamepad1.right_stick_y *5);
//
//            telemetry.addData("LeftJoyStickPosition", gamepad1.left_stick_y);
//            telemetry.addData("RightJoyStickPosition", gamepad1.right_stick_y);
//            idle();
//        }
//    }
//}
