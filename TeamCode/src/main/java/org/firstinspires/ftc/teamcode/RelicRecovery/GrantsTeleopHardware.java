package org.firstinspires.ftc.teamcode.RelicRecovery;

import android.graphics.Color;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorDigitalTouch;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

import java.util.Locale;

public class GrantsTeleopHardware {

    HardwareMap HwMap;

    //In total 6 motors 6 servos  todo HUB - PORT
    //Drive Wheel Components
    public DcMotor left1; //1-0
    public DcMotor left2; //1-1
    public DcMotor right1; //1-2
    public DcMotor right2; //1-3

    //Linear Slide Components
    public DcMotor slider; // 2-0
    public Servo relicClaw; //2-5
    public Servo relicTwist; //2-5

    //Claw Compenentsnbn
    public DcMotor winch; //2-1
    public DcMotor blockRotate;
    public Servo clawBottom; //2-3
    public Servo clawTop; //2-4
    public CRServo topLeft;
    public CRServo topRight;
    public CRServo bottomLeft;
    public CRServo bottomRight;
    //public DistanceSensor glyphSensor;
    public ColorSensor sensorColor;
    public DistanceSensor sensorDistance;
    public DistanceSensor bottomGrab;
    public ColorSensor blockFlat;

    //Jewel Arm Components
    public Servo rotateArm; //2-0
    public Servo lowerArm; //2-1
    public ColorSensor jewelArm;

    BNO055IMU imu;

    DigitalChannel topLimit;
    DigitalChannel winchLimit;
    DigitalChannel clawLimit;
    DigitalChannel liftlimita;
    DigitalChannel liftlimitb;
    DigitalChannel blockCheck;

    Orientation angles;
    Acceleration gravity;

    //Vuforia Information
    public VuforiaLocalizer vuforia;

    //Vuforia
    double tX; //X value extracted from the offset of the target relative to the robot
    double tY; //Y value extracted from the offset of the target relative to the robot
    double tZ; //Z value extracted from the offset of the target relative to the robot
    //--------------------------------------------------------------------------------------------------------
    double rX; //X value extractecd from the rotational componenets of the target relative to the robot
    double rY; //Y value extractecd from the rotational componenets of the target relative to the robot
    double rZ; //Z value extractecd from the rotational componenets of the target relative to the robot

    //BLOCK CLAW CONSTANTS
    public final static double BLOCK_CLAW_OPEN_TOP = 0.67; //0.75
    public final static double BLOCK_CLAW_CLOSED_TOP = 0.51; //0.89
    public final static double BLOCK_CLAW_OPEN_BOTTOM = 0.38; //Open TODO these are named wrongly
    public final static double BLOCK_CLAW_CLOSED_BOTTOM = 0.8;//Closed 0.587
    public final static double BLOCK_CLAW_LIMIT_BOTTOM = 0.37;
    public final static double BLOCK_CLAW_LIMITO_BOTTOM = 0.42;
    //Halfway

    //RELIC CLAW CONSTANTS
    public final static double RELIC_CLAW_OPENED = 0.7;
    //public final static double RELIC_CLAW_MIDDLE = 0.2;
    public final static double RELIC_CLAW_CLOSED = 0.3;
    public final static double RELIC_TWIST_DOWN = 0.01;
    public final static double RELIC_TWIST_UP = 0.99;

    //JEWEL ARM CONSTANTS
    public final static double JEWEL_ARM_UP = 0.85;
    public final static double JEWEL_ARM_SLANTED = 0.59;
    public final static double JEWEL_ARM_DOWN = 0.235; //0.235
    public final static double ROTATE_DOUBLECHECK = 0.15;
    public final static double ROTATE_RIGHT = 0.5;
    public final static double ROTATE_MID = 0.3;
    public final static double ROTATE_LEFT = 0.1;



    public GrantsTeleopHardware() {
        System.out.println("Created new RRHardwarePresets Object!");
    }

    public void init(HardwareMap hwm) {
        //Mappings.
        HwMap = hwm;
        //Drive Motors
        left1 = HwMap.dcMotor.get("left1");
        left2 = HwMap.dcMotor.get("left2");
        right1 = HwMap.dcMotor.get("right1");
        right2 = HwMap.dcMotor.get("right2");

        //Linear Slide
        slider = HwMap.dcMotor.get("slider");
        relicClaw = HwMap.servo.get("relicClaw");
        relicTwist = HwMap.servo.get("relicTwist");

        //Claw
        winch = HwMap.dcMotor.get("winch");
        blockRotate = HwMap.dcMotor.get("blockRotate");
        clawTop = HwMap.servo.get("clawTop");
        clawBottom = HwMap.servo.get("clawBottom");
        topLeft = HwMap.crservo.get("topLeft");
        topRight = HwMap.crservo.get("topRight");
        bottomLeft = HwMap.crservo.get("bottomLeft");
        bottomRight = HwMap.crservo.get("bottomRight");
        //glyphSensor = HwMap.get(DistanceSensor.class, ("glyphSensor"));
        sensorDistance = HwMap.get(DistanceSensor.class, "sensor_color_distance");
        bottomGrab = HwMap.get(DistanceSensor.class, "bottomGrab");
        blockFlat = HwMap.colorSensor.get("blockFlat");

        //Jewel Smacker
        rotateArm = HwMap.servo.get("rotateArm");
        lowerArm = HwMap.servo.get("lowerArm");
        jewelArm = HwMap.colorSensor.get("jewelArm");

        imu = HwMap.get(BNO055IMU.class, "imu");
        topLimit = HwMap.get(DigitalChannel.class, "topLimit");
        winchLimit = HwMap.get(DigitalChannel.class, "winchLimit");
        clawLimit = HwMap.get(DigitalChannel.class, "clawLimit");
        liftlimita = HwMap.get(DigitalChannel.class, "liftlimita");
        liftlimitb = HwMap.get(DigitalChannel.class, "liftlimitb");
        blockCheck = HwMap.get(DigitalChannel.class, "blockCheck");
        // set the digital channel to input.
        topLimit.setMode(DigitalChannel.Mode.INPUT);


        //DC Motor directions.
        left1.setDirection(DcMotorSimple.Direction.FORWARD);
        left2.setDirection(DcMotorSimple.Direction.FORWARD);
        right1.setDirection(DcMotorSimple.Direction.REVERSE);
        right2.setDirection(DcMotorSimple.Direction.REVERSE);
        winch.setDirection(DcMotorSimple.Direction.FORWARD);
        slider.setDirection(DcMotorSimple.Direction.FORWARD);

        left1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        winch.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Sensor LED control.
        jewelArm.enableLed(false);

    }

    public void initServoPositions() {
        this.relicTwist.setPosition(RELIC_TWIST_UP);
        this.relicClaw.setPosition(RELIC_CLAW_OPENED);
        this.clawBottom.setPosition(BLOCK_CLAW_OPEN_BOTTOM);
        this.clawTop.setPosition(BLOCK_CLAW_OPEN_TOP);
        this.rotateArm.setPosition(ROTATE_MID);
        this.lowerArm.setPosition(JEWEL_ARM_UP);
    }

    //Takes power and distance to rotate and "CW" clockwise or "CCW" as directional input.
    public void turnDirection(double power, int distance, String direction) {
        //Resets encoders by setting to STOP_AND_RESET_ENCODER mode.
        setRunMode("STOP_AND_RESET_ENCODER");
        setRunMode("RUN_TO_POSITION");
        if (direction.equals("CCW")) {//Left
            this.left1.setTargetPosition(distance);
            this.left2.setTargetPosition(distance);
            this.right1.setTargetPosition(-distance);
            this.right2.setTargetPosition(-distance);
        } else if (direction.equals("CW")) {//Right
            this.left1.setTargetPosition(-distance);
            this.left2.setTargetPosition(-distance);
            this.right1.setTargetPosition(distance);
            this.right2.setTargetPosition(distance);
        }
        setMotorPower(power);
        //Waits while turning.
        while (anyMotorsBusy()) {
            //Spinning
            //Waiting while turning.
        }
        //Stop motors.
        setMotorPower(0.0);
        //Sets mode back to RUN_USING_ENCODER
        setRunMode("RUN_USING_ENCODER");
    }

    //Drives forward a certain distance at a certain speed. Only use if no intention to interrupt.
    public void driveForwardSetDistance(double power, int distance) {
        //Resets encoders by setting to STOP_AND_RESET_ENCODER mode.
        setRunMode("STOP_AND_RESET_ENCODER");
        //Sets target distance. Set to negative distance because motor was running backwards.
        setAllTargetPositions(-distance);
        //Sets to RUN_TO_POSITION mode
        setRunMode("RUN_TO_POSITION");
        //Sets power for DC Motors.
        setMotorPower(power);
        //Waits while driving to position.
        while (left1.isBusy() & right1.isBusy()) {
            // Spinning.
            // Waiting for robot to arrive at destination.
        }
        //Stops driving by setting power to 0.0.
        setMotorPower(0.0);
        //Sets back to RUN_USING_ENCODER mode.
        setRunMode("RUN_USING_ENCODER");
    }

    //Sets the run mode of all DC motors. Test is this works in both autonomous and teleOp modes.
    public void setRunMode(String input) {
        if (input.equals("STOP_AND_RESET_ENCODER")) {
            this.left1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            this.left2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            this.right1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            this.right2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            this.slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        if (input.equals("RUN_WITHOUT_ENCODER")) {
            this.left1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            this.left2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            this.right1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            this.right2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            this.slider.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        if (input.equals("RUN_USING_ENCODER")) {
            this.left1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            this.left2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            this.right1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            this.right2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            this.slider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        if (input.equals("RUN_TO_POSITION")) {
            this.left1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            this.left2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            this.right1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            this.right2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            this.slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }

    //Returns TRUE if any drive motors are busy and FALSE if not.
    public boolean anyMotorsBusy() {
        if (this.left1.isBusy() || this.left2.isBusy() || this.right1.isBusy() || this.right2.isBusy()) {
            return (true);
        } else {
            return (false);
        }
    }

    //Sets all drive motor power.
    public void setMotorPower(double power) {
        this.left1.setPower(power);
        this.left2.setPower(power);
        this.right1.setPower(power);
        this.right2.setPower(power);
    }

    //Sets all motors target position.
    public void setAllTargetPositions(int distance) {
        left1.setTargetPosition(distance);
        left2.setTargetPosition(distance);
        right1.setTargetPosition(distance);
        right2.setTargetPosition(distance);
    }

    public void moveServo(Servo targetServo, double targetPosition, int steps, long timeInMilli) {
        //Total distance to travel.
        double distanceToTravel = Math.abs(targetServo.getPosition() - targetPosition);
        //Unit conversion to nanoseconds.
        long time = timeInMilli * 1000000;
        //Per Step values.
        //double distanceToTravelPerStep = (distanceToTravel / steps);
        long timePerStep = time / steps;
        //Loops number of steps.
        double distanceToTravelPerStep;
        if (targetPosition - targetServo.getPosition() >= 0) {
            distanceToTravelPerStep = (distanceToTravel / steps);
        } else {
            distanceToTravelPerStep = (distanceToTravel / steps) * -1;
        }
        for (int counter = 0; counter < steps; counter++) {
            double initialTime = System.nanoTime();
            double currentPosition = targetServo.getPosition(); //Gets current arm position.
            //if(movementFlag == 0) {
            targetServo.setPosition(currentPosition + distanceToTravelPerStep);//Moves the arm.
            while ((System.nanoTime() - initialTime) < timePerStep) {
                //Wait.
            }
        }
    }

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees) {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
}