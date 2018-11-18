package org.firstinspires.ftc.teamcode.RoverRuckus;//package org.firstinspires.ftc.teamcode.RoverRuckus;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "RoverTeleop", group = "Teleop")
public class RoverQualifierTeleop extends LinearOpMode{
    public RoverHardware robot = new RoverHardware();

    public void runOpMode(){
        robot.init(hardwareMap);

        while(opModeIsActive()){
            robot.left1.setPower(gamepad1.left_stick_y);
            robot.right1.setPower(gamepad1.right_stick_y);

            if(gamepad1.dpad_up){
                robot.hang.setPower(0.8);
            } else if(gamepad1.dpad_down){
                robot.hang.setPower(-0.8);
            }
            idle();
        }
    }
}