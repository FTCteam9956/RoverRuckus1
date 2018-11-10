package org.firstinspires.ftc.teamcode.RoverRuckus;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Locale;

public class RoverHardware {

    HardwareMap HwMap;

    //Drive Motors
    DcMotor left1;
    // DcMotor left2;
    DcMotor right1;
    //DcMotor right2;
//    DcMotor launcher;
    //Hanging Mechanism
    DcMotor hang;
    //bopper
//    DcMotor bop;
    //dropper
//    Servo drop;

    //Create Sensors
//    DistanceSensor senseOBJ;
    DistanceSensor sensorRange;

    //Limit Switches
    DigitalChannel LimitA;
    DigitalChannel LimitB;

    //Create Gyro
    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;

    public RoverHardware() {
        System.out.println("Created new RRHardwarePresets Object!");
    }


    public void init(HardwareMap hwm) {

        HwMap = hwm;
        //Drive Motors
        left1 = HwMap.dcMotor.get("left1");
        // left2 = HwMap.dcMotor.get("left2");
        right1 = HwMap.dcMotor.get("right1");
        // right2 = HwMap.dcMotor.get("right2");
        //Hanging Motor
        hang = HwMap.dcMotor.get("hang");
//        senseOBJ = HwMap.get(DistanceSensor.class, "senseOBJ");
        imu = HwMap.get(BNO055IMU.class, "imu");
        //Launching Motor
//        launcher = HwMap.dcMotor.get("launcher");
        //bopper
//        bop = HwMap.dcMotor.get("bop");
        //dropper
//        drop = HwMap.servo.get("drop");

        //Limit Switches
        LimitA = HwMap.digitalChannel.get("limitA");
        //LimitB = HwMap.digitalChannel.get("limitB");

        //Range Sensors
      // sensorRange = HwMap.get(DistanceSensor.class, "sensor_range");

        //Set DcMotor Directions and Behaviors
        left1.setDirection(DcMotorSimple.Direction.FORWARD);
        //left2.setDirection(DcMotorSimple.Direction.REVERSE);
        right1.setDirection(DcMotorSimple.Direction.REVERSE);
        //right2.setDirection(DcMotorSimple.Direction.FORWARD);
//        hang.setDirection(DcMotorSimple.Direction.FORWARD);

        left1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //left2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //right2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hang.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //bopper
        //bop.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //dropper
    }
//
    public void initServoPositions() {
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

