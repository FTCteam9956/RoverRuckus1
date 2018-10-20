package org.firstinspires.ftc.teamcode.RelicRecovery;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RelicRecovery.GrantsTeleopHardware;


@TeleOp(name = "Sensor Values Test", group = "Teleop")
@Disabled
public class SENSORTEST extends LinearOpMode {

    public void runOpMode(){
        GrantsTeleopHardware robot = new GrantsTeleopHardware();

        robot.init(hardwareMap);
        waitForStart();

        while(opModeIsActive()){

            robot.lowerArm.setPosition(robot.JEWEL_ARM_DOWN);

            telemetry.addData("sensor red", robot.jewelArm.red());
            telemetry.addData("sensor blue", robot.jewelArm.blue());
            telemetry.addData("sensor green", robot.jewelArm.green());
            telemetry.addData("total luminosity", robot.jewelArm.alpha());
            telemetry.addData("total color", robot.jewelArm.argb());

            idle();
        }
    }
}
