package org.firstinspires.ftc.teamcode.VelocityVortex;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;

public class VVNewHardware {
    public DcMotor left;
    public DcMotor right;
    public DcMotor loader;
    public DcMotor shooter;
    public DcMotor intake;
    public DcMotor winch;
    public DcMotor spinner;
    public Servo leftArm;
    public Servo rightArm;

    public ColorSensor beacon;
    public ColorSensor botClose;
    public ColorSensor botFar;
    public ColorSensor ballCheck;

    public static final double RIGHT_OUT = 1;
    public static final double RIGHT_IN = 0;
    public static final double LEFT_OUT = 0;
    public static final double LEFT_IN = 1;

    public static final double ENDGAME_PERCENT = 0.75;
    public static final double SHOOTER_SPEED = 0.88;//0.6
    public static final double INTAKE_SPEED = 0.7;
    public static final double LOADER_SPEED = 0.45;//0.5
    public static final double SPINNER_SPEED = 0.5;

    public static final int BALL_THRESHOLD = 5;

    public static final int LOADER_ONE_BALL = 450; //440
    //    public static final int LOADER_INITIAL = 50;
    public static final int MAX_PARTICLES = 4;

    HardwareMap hwMap;

    public VVNewHardware() {}

    public void init(HardwareMap hwm) {
        hwMap = hwm;

        left = hwMap.dcMotor.get("left");
        right = hwMap.dcMotor.get("right");
        loader = hwMap.dcMotor.get("loader");
        shooter = hwMap.dcMotor.get("shooter");
        intake = hwMap.dcMotor.get("intake");
        winch = hwMap.dcMotor.get("winch");
        spinner = hwMap.dcMotor.get("spinner");
        leftArm = hwMap.servo.get("left arm");
        rightArm = hwMap.servo.get("right arm");

        beacon = hwMap.colorSensor.get("beacon");
//        beacon.setI2cAddress(I2cAddr.create8bit(0x3c));
        botClose = hwMap.colorSensor.get("bot close");
        botClose.setI2cAddress(I2cAddr.create8bit(0x3a));
        botFar = hwMap.colorSensor.get("bot far");
        botFar.setI2cAddress(I2cAddr.create8bit(0x30));
        ballCheck = hwMap.colorSensor.get("ball check");
        ballCheck.setI2cAddress(I2cAddr.create8bit(0x32));

        left.setDirection(DcMotorSimple.Direction.FORWARD);
        right.setDirection(DcMotorSimple.Direction.REVERSE);
        loader.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        winch.setDirection(DcMotorSimple.Direction.FORWARD);
        spinner.setDirection(DcMotorSimple.Direction.FORWARD);

        winch.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        loader.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        spinner.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftArm.setPosition(LEFT_IN);
        rightArm.setPosition(RIGHT_IN);

        beacon.enableLed(false);
        botClose.enableLed(false);
        botFar.enableLed(false);
        ballCheck.enableLed(false);
    }
}

