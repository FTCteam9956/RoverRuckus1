package org.firstinspires.ftc.teamcode.RelicRecovery;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Range", group = "Teleop")
@Disabled
public class AcousticSensors extends LinearOpMode{

    public static int RealRange = 0;
    public void runOpMode(){

        ModernRoboticsI2cRangeSensor wallSensor;
        waitForStart();
        wallSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "wallSensor");

        while(opModeIsActive()){

            if(wallSensor.rawUltrasonic() == 255){
                telemetry.addData("UltraSonic", RealRange);
            }else if(wallSensor.rawUltrasonic() < 255){
                RealRange = wallSensor.rawUltrasonic();
                telemetry.addData("UltraSonic", RealRange);
            }
//            telemetry.addData("raw ultrasonic", wallSensor.rawUltrasonic());
//            telemetry.addData("raw optical", wallSensor.rawOptical());
            telemetry.update();
            idle();
        }
    }
}
