package org.firstinspires.ftc.teamcode.RoverRuckus;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class RoverHardware{

    HardwareMap HwMap;

    //Drive Motors
   // DcMotor left1;
   // DcMotor left2;
   // DcMotor right1;
   // DcMotor right2;
    DcMotor launcher;
    //Hanging Mechanism
  //  DcMotor hang;

    //sensors
    DistanceSensor senseOBJ;

    BNO055IMU imu;

    public RoverHardware() {System.out.println("Created new RRHardwarePresets Object!");}


    public void init(HardwareMap hwm){

        HwMap = hwm;
        //Drive Motors
       // left1 = HwMap.dcMotor.get("left1");
       // left2 = HwMap.dcMotor.get("left2");
       // right1 = HwMap.dcMotor.get("right1");
      //  right2 = HwMap.dcMotor.get("right2");
        //Hanging Motor
      //  hang = HwMap.dcMotor.get("hang");
        senseOBJ = HwMap.get(DistanceSensor.class, "senseOBJ");
        imu = HwMap.get(BNO055IMU.class, "imu");
        //Launching Motor
        launcher = HwMap.dcMotor.get("launcher");

        //Set DcMotor Directions and Behaviors
       /* left1.setDirection(DcMotorSimple.Direction.REVERSE);
        left2.setDirection(DcMotorSimple.Direction.REVERSE);
        right1.setDirection(DcMotorSimple.Direction.FORWARD);
        right2.setDirection(DcMotorSimple.Direction.FORWARD);
        hang.setDirection(DcMotorSimple.Direction.FORWARD);

        left1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
       *//// hang.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    public void initServoPositions(){

    }
    public void DriveStraightSetAmount(double power, int encodercounts, int sleepTimeInMillis){
       // left1.setTargetPosition(encodercounts);

    }

}
