package org.firstinspires.ftc.teamcode.RoverRuckus;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "test", group = "TeleOp")
public class encodertest extends LinearOpMode {
    RoverHardware robot = new RoverHardware();

    public void runOpMode(){
        robot.init(hardwareMap);

        waitForStart();
        robot.left1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.right1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.left1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.right1.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.left1.setTargetPosition(1000);
        robot.right1.setTargetPosition(1000);

        robot.left1.setPower(0.1);
        robot.right1.setPower(0.1);

        while(robot.left1.isBusy() && opModeIsActive()){
            telemetry.addData("Left Power", robot.left1.getPower());
            telemetry.addData("Right Power", robot.right1.getPower());
            telemetry.addData("Left Position", robot.left1.getCurrentPosition());
            telemetry.addData("Right Position", robot.right1.getCurrentPosition());
            telemetry.update();
        }
        robot.left1.setPower(0);
        robot.right1.setPower(0);

    }
}
