package org.firstinspires.ftc.teamcode.RoverRuckus;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "QualifierTeleop", group = "Teleop")
public class RoverTeleopQualifier extends LinearOpMode{
//
//    public DcMotor hang;
//    public DcMotor left1;
//    public DcMotor left2;
//    public DcMotor right1;
//    public DcMotor right2;

    public RoverHardware robot = new RoverHardware();

  public int OBJ = 2; // # is in CM

    public void runOpMode(){
        robot.init(hardwareMap);
//        hang = hardwareMap.dcMotor.get("hang");
//        left1 = hardwareMap.dcMotor.get("left1");
//        left2 = hardwareMap.dcMotor.get("left2");
//        right1 = hardwareMap.dcMotor.get("right1");
//        right2 = hardwareMap.dcMotor.get("right2");

//        robot.left1.setDirection(DcMotorSimple.Direction.REVERSE);
//        robot.left2.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while(opModeIsActive()){
            //robot.left1.setPower(gamepad1.left_stick_y * .3);
            //robot.left2.setPower(gamepad1.left_stick_y * .3);
            //robot.right1.setPower(gamepad1.right_stick_y * .3);
            //robot.right2.setPower(gamepad1.right_stick_y * .3);

          if (gamepad1.dpad_up) {
       //       robot.hang.setPower(-1);
          } else if (gamepad1.dpad_down) {
       //      robot.hang.setPower(1);
          } else{
        //      robot.hang.setPower(0);
          }

          if(gamepad1.x) {
                robot.launcher.setPower(1);
                sleep(250);
                robot.launcher.setPower(0);
            }
          }

        }


    }

