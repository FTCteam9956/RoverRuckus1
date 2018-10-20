package org.firstinspires.ftc.teamcode.RoverRuckus;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "QualifierTeleop", group = "Teleop")
public class RoverTeleopQualifier extends LinearOpMode {

    public DcMotor hang;
    public DcMotor left1;
    public DcMotor left2;
    public DcMotor right1;
    public DcMotor right2;

    public RoverHardware robot = new RoverHardware();

    public void runOpMode(){

        hang = hardwareMap.dcMotor.get("hang");
        left1 = hardwareMap.dcMotor.get("left1");
        left2 = hardwareMap.dcMotor.get("left2");
        right1 = hardwareMap.dcMotor.get("right1");
        right2 = hardwareMap.dcMotor.get("right2");

        left1.setDirection(DcMotorSimple.Direction.REVERSE);
        left2.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while(opModeIsActive()){
            left1.setPower(gamepad1.left_stick_y * .2);
            left2.setPower(gamepad1.left_stick_y * .2);
            right1.setPower(gamepad1.right_stick_y * .2);
            right2.setPower(gamepad1.right_stick_y * .2);

          if (gamepad1.dpad_up) {
              hang.setPower(-1);
          } else if (gamepad1.dpad_down) {
              hang.setPower(1);
          } else{
              hang.setPower(0);
          }
        }
    }
}
