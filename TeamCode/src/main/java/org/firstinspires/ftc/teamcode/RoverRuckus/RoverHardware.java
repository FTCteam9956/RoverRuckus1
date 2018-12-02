package org.firstinspires.ftc.teamcode.RoverRuckus;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import java.util.TimerTask;
import java.util.Timer;


import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;
import org.openftc.revextensions2.RevExtensions2;

import java.util.Locale;

public class RoverHardware {



    HardwareMap HwMap;

    //Drive Motors
    public DcMotor left1;
    // DcMotor left2;
    public DcMotor right1;
   //DcMotor right2;
    public DcMotor launcher;
    //Hanging Mechanism
    public DcMotor hang;
    //bopper
    public DcMotor bop;
    //Rotation Mechanism
    public DcMotor rotateMech;
    // Ball Catcher???
    public DcMotor  ballCatch;
    //dropper
    public Servo drop;
    //marker dropper
    public Servo marker;
    //Intake Servo
    public CRServo intake;



    //Create Sensors
    //DistanceSensor senseOBJ;
    public DistanceSensor sensorRange;

    //Color Sensor 'Limit Switches'
    public ColorSensor bottomLimit;
    public ColorSensor upperLimit;
    public ColorSensor cornerSensor;

    //Create Gyro
    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;

    //Exapnsion Hubs
    ExpansionHubEx expansionHub;
    //ExpansionHubEx expansionHub3;

    //Expansion Hub Motors
//    ExpansionHubMotor left,right;


    RevBulkData bulkData;
    AnalogInput a0, a1, a2, a3;
    DigitalChannel d0, d1, d2, d3, d4, d5, d6, d7;

    public final double BOTTOM_INTAKE = 1;
    public final double TOP_INTAKE = 0.3;
    public final double DILBERT_DOWN = 1.0;
    public final double DILBERT_UP = 0.0;

    public RoverHardware() {
        System.out.println("Created new RRHardwarePresets Object!");
    }


    public void init(HardwareMap hwm) {

        HwMap = hwm;
        //Expansion Hubs
        //expansionHub = HwMap.get(ExpansionHubEx.class, "Expansion Hub 2");
        //expansionHub3 = HwMap.get(ExpansionHubEx.class, "Expansion Hub 3");


        //Expansion Hub Motors
//        left = (ExpansionHubMotor) HwMap.dcMotor.get("left1");
//        right = (ExpansionHubMotor) HwMap.dcMotor.get("right1");

        //Drive Motors
        left1 = HwMap.dcMotor.get("left1");
        right1 = HwMap.dcMotor.get("right1");
        //Hanging Motor
        hang = HwMap.dcMotor.get("hang");
        //Launching Motor
        launcher = HwMap.dcMotor.get("launcher");
        //bopper
        bop = HwMap.dcMotor.get("bop");
        //Rotation Mechanism
        rotateMech = HwMap.dcMotor.get("rotate");
        //Ball Catcher
        ballCatch = HwMap.dcMotor.get("ballCatch");
        //dropper
        drop = HwMap.servo.get("drop");
        //marker dropper
        marker = HwMap.servo.get("marker");
        //Intake servo
        intake = HwMap.crservo.get("intake");

        //ColorSensors
        bottomLimit = HwMap.colorSensor.get("bottomLimit");
        upperLimit = HwMap.colorSensor.get("upperLimit");
        cornerSensor = HwMap.colorSensor.get("cornerSensor");

        //Gyro
        imu = HwMap.get(BNO055IMU.class, "imu");

        //Range Sensors
      // sensorRange = HwMap.get(DistanceSensor.class, "sensor_range");

        //Set DcMotor Directions and Behaviors
        left1.setDirection(DcMotorSimple.Direction.FORWARD);
        right1.setDirection(DcMotorSimple.Direction.REVERSE); //should be the other way
        hang.setDirection(DcMotorSimple.Direction.FORWARD);
        bop.setDirection(DcMotorSimple.Direction.FORWARD);
        launcher.setDirection(DcMotorSimple.Direction.FORWARD);
        rotateMech.setDirection(DcMotorSimple.Direction.FORWARD);

        left1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hang.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bop.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rotateMech.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
//
    public void initServoPositions() {
        drop.setPosition(TOP_INTAKE);
    }

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees) {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    public void setRunMode(String input) {
        if (input.equals("STOP_AND_RESET_ENCODER")) {
            this.left1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            this.right1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        if (input.equals("RUN_WITHOUT_ENCODER")) {
            this.left1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            this.right1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        if (input.equals("RUN_USING_ENCODER")) {
            this.left1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            this.right1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        if (input.equals("RUN_TO_POSITION")) {
            this.left1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            this.right1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }

    //Returns TRUE if any drive motors are busy and FALSE if not.
    public boolean anyMotorsBusy() {
        if (this.left1.isBusy() || this.right1.isBusy()) {
            return (true);
        } else {
            return (false);
        }
    }

    //Sets all drive motor power.
    public void setMotorPower(double power) {
        this.left1.setPower(power);
        this.right1.setPower(power);
    }

    //Sets all motors target position.
    public void setAllTargetPositions(int distance) {
        left1.setTargetPosition(distance);
        right1.setTargetPosition(distance);
    }

    public void driveForwardSetDistance(double power, int distance) {
        //Resets encoders by setting to STOP_AND_RESET_ENCODER mode.
        setRunMode("STOP_AND_RESET_ENCODER");
        //Sets target distance. Set to negative distance because motor was running backwards.
        setAllTargetPositions(distance);
        //Sets to RUN_TO_POSITION mode
        setRunMode("RUN_TO_POSITION");
        //Sets power for DC Motors.
        setMotorPower(power);
        //Waits while driving to position.
        while (left1.isBusy() & right1.isBusy()) {
            // Spinning.
            // Waiting for robot to arrive at destination.
        }
    }
}

