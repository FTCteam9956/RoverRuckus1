package org.firstinspires.ftc.teamcode.RoverRuckus;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Locale;

public class RoverHardware {

    HardwareMap HwMap;

    //Initialize Motors
    public DcMotor left1;
    public DcMotor right1;
    public DcMotor hang;
    public DcMotor blockArm; //Also known as bopper
    public DcMotor lazySusan;
//    public DcMotor launcher;
//    public Servo drop;

    //Create Sensors
    DistanceSensor sensorRange;

    //Create Limit Switches
//    ColorSensor top;
//    ColorSensor bottom;

    //Create Gyro
    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;

    public void init(HardwareMap hwm){

        HwMap = hwm;

        //Map Motors
        left1 = HwMap.dcMotor.get("left1");
        right1 = HwMap.dcMotor.get("right1");
        hang = HwMap.dcMotor.get("hang");
        //launcher = HwMap.dcMotor.get("launcher");
        blockArm = HwMap.dcMotor.get("blockArm");
        lazySusan = HwMap.dcMotor.get("lazySusan");

        //Map Coor sensors
//        top = HwMap.colorSensor.get("top");
//        bottom = HwMap.colorSensor.get("bottom");

        //Map IMU
        imu = HwMap.get(BNO055IMU.class, "imu");

        //Map Range Sensors
        //sensorRange = HwMap.get(DistanceSensor.class, "sensorRange");

        //Servo
        //drop = HwMap.servo.get("drop");

        //Set Motor Directions and Behaviors
        left1.setDirection(DcMotorSimple.Direction.FORWARD);
        right1.setDirection(DcMotorSimple.Direction.REVERSE);
        hang.setDirection(DcMotorSimple.Direction.FORWARD);
//        launcher.setDirection(DcMotorSimple.Direction.FORWARD);
        blockArm.setDirection(DcMotorSimple.Direction.FORWARD);
        lazySusan.setDirection(DcMotorSimple.Direction.FORWARD);

        left1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hang.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        blockArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lazySusan.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    String formatAngle(AngleUnit angleUnit, double angle){
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }
    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
}
