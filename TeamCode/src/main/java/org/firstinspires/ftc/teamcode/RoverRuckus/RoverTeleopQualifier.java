package org.firstinspires.ftc.teamcode.RoverRuckus;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Locale;

@TeleOp(name = "QualifierTeleop", group = "Teleop")
public class RoverTeleopQualifier extends LinearOpMode{

    public RoverHardware robot = new RoverHardware();

    public float leftPower;
    public float rightPower;
    public float xValue;
    public float yValue;

    public void runOpMode(){
        robot.init(hardwareMap);

        waitForStart();

        while(opModeIsActive()){

            //Arcade Drive
            yValue = gamepad1.left_stick_y;
            xValue = gamepad1.left_stick_x;

            leftPower =  yValue - xValue;
            rightPower = yValue + xValue;

            robot.left1.setPower(Range.clip(leftPower, -1.0, 1.0));
            robot.right1.setPower(Range.clip(rightPower, -1.0, 1.0));

            //Move block intake in and out
            robot.bop.setPower(gamepad1.right_stick_y * 0.5);

            //Hanging Mechanism
          if (gamepad1.dpad_up && robot.upperLimit.red() > 300){
              robot.hang.setPower(1);
          } else if (gamepad1.dpad_down && robot.bottomLimit.red() < 100){
             robot.hang.setPower(-1);
          } else{
              robot.hang.setPower(0);
          }

            //Shoots blocks
          if (gamepad1.b){
                robot.launcher.setPower(-1);
                sleep(250);
                robot.launcher.setPower(0);
            }

            //Sets Servo Position to Top
          if (gamepad1.y) {
                robot.drop.setPosition(robot.TOP_INTAKE);
          }

            //Sets Servo Position to Bottom
          if(gamepad1.a){
              robot.drop.setPosition(robot.BOTTOM_INTAKE);
          }

          //Rotates the Bopper
          if(gamepad1.right_trigger >= 0.5){
              robot.rotateMech.setPower(-gamepad1.right_trigger * 0.5);
          }
          else if (gamepad1.left_trigger >= 0.5){
              robot.rotateMech.setPower(gamepad1.left_trigger * 0.5);
          }
          else {
              robot.rotateMech.setPower(0);
          }

          //Ball Catching Controls
            if(gamepad1.right_bumper){
              robot.ballCatch.setPower(0.75);
            }
            else if(gamepad1.left_bumper){
              robot.ballCatch.setPower(-0.75);
            }
            else{
              robot.ballCatch.setPower(0);
            }


            //Telemetry Section
//            telemetry.addData("Distance (cm)", //Checks what the distance sensor on the launcher sees
//            String.format(Locale.US, "%.02f", robot.senseOBJ.getDistance(DistanceUnit.CM)));

//            telemetry.addData("Left Power", leftPower);
//            telemetry.addData("Right Power", rightPower);
//            telemetry.addData("Gamepad Tigger", gamepad1.right_trigger);
//            telemetry.addData("upper red", robot.upperLimit.red());
//            telemetry.addData("upper blue", robot.upperLimit.blue());
//            telemetry.addData("bottom red", robot.bottomLimit.red());
//            telemetry.addData("bottom blue", robot.bottomLimit.blue());
//            telemetry.addData("stick", "  y=" + yValue + "  x=" + xValue);
//            telemetry.addData("power", "  left=" + leftPower + "  right=" + rightPower);
            telemetry.addData("Arm power", robot.bop.getPower());
            telemetry.addData("Arm position", robot.bop.getCurrentPosition());
            telemetry.update();
        }
    }
}