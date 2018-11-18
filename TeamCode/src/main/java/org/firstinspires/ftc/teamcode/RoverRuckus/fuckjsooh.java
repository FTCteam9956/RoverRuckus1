package org.firstinspires.ftc.teamcode.RoverRuckus;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;

@TeleOp(name = "s", group = "Teleop")
public class fuckjsooh extends LinearOpMode {

    public DigitalChannel topLimit;
    public void runOpMode(){
        while (opModeIsActive()) {
            if(topLimit.getState() == true) {
                telemetry.addData("status", "Pressed");
            } else if(topLimit.getState() == false){
                telemetry.addData("status", "Not Pressed");
            }
        }
    }
}
