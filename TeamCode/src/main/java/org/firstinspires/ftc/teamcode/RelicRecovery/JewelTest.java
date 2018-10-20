package org.firstinspires.ftc.teamcode.RelicRecovery;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RelicRecovery.GrantsTeleopHardware;


@TeleOp(name = "Jewel Test", group = "Teleop")
@Disabled
public class JewelTest extends LinearOpMode {
    GrantsTeleopHardware robot = new GrantsTeleopHardware();

    public void runOpMode(){
        robot.init(hardwareMap);

        waitForStart();

        while(opModeIsActive()){
            if(gamepad1.a){
                robot.rotateArm.setPosition(robot.ROTATE_MID);
            }
            else if(gamepad1.b){
                robot.rotateArm.setPosition(robot.ROTATE_RIGHT);
            }
            else if(gamepad1.x){
                robot.rotateArm.setPosition(robot.ROTATE_LEFT);
            }
            idle();
        }
    }
}
