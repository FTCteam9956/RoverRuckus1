package org.firstinspires.ftc.teamcode.RoverRuckus;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Locale;

@TeleOp(name = "QualifierTeleop", group = "Teleop")
public class RoverTeleopQualifier extends LinearOpMode{

    public RoverHardware robot = new RoverHardware();

  public int OBJ = 2; // # is in CM
    public float leftPower;
    public float rightPower;

    public void runOpMode(){
        robot.init(hardwareMap);

        waitForStart();

        while(opModeIsActive()){

            //Drive Motors
            leftPower = (gamepad1.left_stick_y + gamepad1.left_stick_x);
            rightPower = (gamepad1.left_stick_y - gamepad1.left_stick_x);
            //Tank Drive
            robot.left1.setPower(-leftPower / 1.25);

            robot.right1.setPower(-rightPower / 1.25);

            //Moves the Bopper In and Out
            robot.bop.setPower(gamepad1.right_stick_y * 0.5);

            // Hang Mechanism
          if (gamepad1.dpad_up && robot.upperLimit.red() < 110){
              robot.hang.setPower(-1);
          } else if (gamepad1.dpad_down && robot.bottomLimit.red() < 110){
             robot.hang.setPower(1);
          } else{
              robot.hang.setPower(0);
          }
            //Activates the Launcher
          if (gamepad1.b){
                robot.launcher.setPower(-1);
                sleep(250);
                robot.launcher.setPower(0);
            }
            //Sets Servo Position to Top
          if (gamepad1.y) {
                robot.drop.setPosition(0.3);
          }
            //Sets Servo Position to Bottom
          if(gamepad1.a){
              robot.drop.setPosition(0.85);
          }
          //Rotates the Bopper
          if(gamepad1.right_trigger >= 0.5){
              robot.rotateMech.setPower(gamepad1.right_trigger * 0.5);
          }
          else if (gamepad1.left_trigger >= 0.5){
              robot.rotateMech.setPower(-gamepad1.left_trigger * 0.5);
          }
          else {
              robot.rotateMech.setPower(0);
          }
            //Telemetry Section
//            telemetry.addData("Distance (cm)", //Checks what the distance sensor on the launcher sees
//                    String.format(Locale.US, "%.02f", robot.senseOBJ.getDistance(DistanceUnit.CM)));

            telemetry.addData("Left Power", leftPower);
            telemetry.addData("Right Power", rightPower);
            telemetry.addData("Gamepad Tigger", gamepad1.right_trigger);
        }
    }
}