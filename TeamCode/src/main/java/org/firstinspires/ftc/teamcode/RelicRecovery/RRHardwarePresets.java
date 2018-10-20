//RRHardwarePresets.java

package org.firstinspires.ftc.teamcode.RelicRecovery;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.RelicRecovery.Position;

public class RRHardwarePresets{

    //Hardware Map.
    HardwareMap HwMap;

    //DcMotors
    public DcMotor left1;
    public DcMotor left2;
    public DcMotor right1;
    public DcMotor right2;
    public DcMotor turretMotor;
    public DcMotor winchMotor;

    //Arm Servos
    public Servo elbow;
    public Servo wrist;
    public DcMotor shoulder;

    //Servos
    public Servo claw;
    public Servo clawTwist;
    public Servo jewelArm;

    //Sensors
    public ColorSensor jewelSensor;
    public ColorSensor floorSensor;
    public BNO055IMU imu1;
    public BNO055IMU imu2;

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



    // Jewel Arm Constants
    public final double JEWEL_ARM_UP = 0.01;
    public final double JEWEL_ARM_DOWN = 0.85;

    //Set Position Constants
    public final double ELBOW_UNFOLDED = 0.30;
    public final double ELBOW_FOLDED = 1.00;
    public final double WRIST_UNFOLDED = 0.7;
    public final double WRIST_FOLDED = 0.0;

    //Positions
    public Position redTurnLeft = new Position(-66, 0.73, 0.29);
    public Position redTurnCenter = new Position(-135, 0.66, 0.29);
    public Position redTurnRight = new Position(-135, 0.66, 0.30);

    //public Position redStraightLeft = new Position();
    //public Position redStraightCenter = new Position();
    //public Position redStraightRight = new Position();

    //public Position blueStraightLeft = new Position();
    //public Position blueStraightCenter = new Position();
    //public Position blueStraightRight = new Position();

    //public Position blueTurnLeft = new Position();
    //public Position blueTurnCenter = new Position();
    //public Position blueTurnRight = new Position();

    //Claw Constants
    public final double CLAW_CLOSED = 0.00;
    public final double CLAW_OPENED = 0.57;
    public final double CLAW_MID = 0.3;
    public final double TWIST_UP = 0.562;
    public final double TWIST_DOWN = 0.928;

    public final int WINCH_DOWN = 10;
    //Constants for placing block and autonomous
    public final double TELBOW_CENTER = 0.8;
    public final double TWRIST_CENTER = 0.6;
    public final double TELBOW_LEFT = 0.6;
    public final double TWRIST_LEFT = 0.6;
    public final double TELBOW_RIGHT = 0.64;
    public final double TWRIST_RIGHT = 0.3;

    //These are the Positions for red turn left
    public final double REDTURN_WRIST_LEFT = .1400;
    public final double REDTURN_ELBOW_LEFT = .7230;
    public final int REDTURN_SHOULDER_LEFT = 321;

    //These are the postions for the red turn right
    public final double REDTURN_WRIST_RIGHT = .30;
    public final double REDTURN_ELBOW_RIGHT = .66;
    public final double REDTURN_SHOULDER_RIGHT = -135;

    //These are the Positions for the red turn center
    public final double REDTURN_WRIST_CENTER = .29;
    public final double REDTURN_ELBOW_CENTER = .66;
    public final double REDTURN_SHOULDER_CENTER = -135;

    public final double ELBOW_CENTER = 0.4;
    public final double WRIST_CENTER = 0.3;
    public final double ELBOW_LEFT = 0.35;
    public final double WRIST_LEFT = 0.25;
    public final double ELBOW_RIGHT = 0.5;
    public final double WRIST_RIGHT = 0.4;

    public final int DRIVE_OFF_STONE = -800;
    public final int DRIVE_INTO_STONE = 120;

    public final int TURRET_FOR_WALL = 560;
    public final int TURRET_FOR_RELIC = -625;

    public final double ELBOW_RELIC = 0.9;
    public final double WRIST_RELIC = 0.4;

    //Need to get these values correct for followLine() to work.
    public final double FLOOR_COLOR = 0.0;
    public final double RED_LINE_COLOR = 0.0;
    public final double BLUE_LINE_COLOR = 0.0;

    //Constructor
    public RRHardwarePresets(){
        System.out.println("Created new RRHardwarePresets Object!");
    }

    public void init(HardwareMap hwm) {

        //Mappings.
        HwMap = hwm;
        left1 = HwMap.dcMotor.get("left1");
        left2 = HwMap.dcMotor.get("left2");
        right1 = HwMap.dcMotor.get("right1");
        right2 = HwMap.dcMotor.get("right2");

        turretMotor = HwMap.dcMotor.get("turretMotor");
        winchMotor = HwMap.dcMotor.get("winchMotor");

        claw = HwMap.servo.get("claw");
        clawTwist = HwMap.servo.get("clawTwist");

        jewelArm = HwMap.servo.get("jewelArm");

        jewelSensor = HwMap.colorSensor.get("jewelSensor");
        floorSensor = HwMap.colorSensor.get("floorSensor");

        elbow = HwMap.servo.get("elbow");
        wrist = HwMap.servo.get("wrist");
        shoulder = HwMap.dcMotor.get("shoulder");
        //AnalogInput potentiometer = HwMap.analogInput.get("pMeter");

        //DC Motor directions.
        left1.setDirection(DcMotorSimple.Direction.FORWARD);
        left2.setDirection(DcMotorSimple.Direction.FORWARD);
        right1.setDirection(DcMotorSimple.Direction.REVERSE);
        right2.setDirection(DcMotorSimple.Direction.REVERSE);
        winchMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        //DC Motor stop behavior.
        left1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        winchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shoulder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Sensor LED control.
        jewelSensor.enableLed(false);
        floorSensor.enableLed(false);
    }

        //IMU Initialization parameters.
//        BNO055IMU.Parameters IMUParameters = new BNO055IMU.Parameters();
//        IMUParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
//        IMUParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
//        IMUParameters.calibrationDataFile = "BNO055IMUCalibration.json"; //See the calibration sample opmode.
//        IMUParameters.loggingEnabled = true;
//        IMUParameters.loggingTag = "IMU";
//        IMUParameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
//
//        //Giving parameters to create imu1.
//        imu1 = HwMap.get(BNO055IMU.class, "imu1");
//        imu1.initialize(IMUParameters);
//
//        //Giving parameters to create imu2.
//        imu2 = HwMap.get(BNO055IMU.class, "imu2");
//        imu2.initialize(IMUParameters);
//
//
//        //Vuforia Initialization parameters.
//        OpenGLMatrix lastLocation = null;
//        int cameraMonitorViewId = HwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", HwMap.appContext.getPackageName()); //Sets camera feed to display on phone.
//        VuforiaLocalizer.Parameters VuforiaParameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId); // If you want to deactivate the Camera Monitor View, to save power. You can just not pass in cameraMonitorViewID.
//        VuforiaParameters.vuforiaLicenseKey = "AU0kxmH/////AAAAGV4QPVzzlk6Hl969cSL2pmM4F6TuzhWZS/dKbY45MEzS31OYJxLbKewdt1CSFrmpvrpPnIYZyBJt3kFRJQCtEXet0LHd2KtBB5NsDTuBADfgIsQk+7TSWSTFDjSi8SpKaXtAjZPKePwGDaIKf5VK6mRBYaWxqTHpZFBlelejLHxib8qweOFrJjKTsbgsb2pwVNFhDeJabbI5aed8JSI8LxHs0368ezQfnCz3UK9u8pC1DkKgcwdgoJ0OXBKChXB4v2lEnIrQf7ROYcPtVuRJJ5/prBoyfR11pvp69iCA25Cttz9xVsdZ9VliuQJ4UO37Hzhz1dB2SPnxTQQmCJMDoDKqe3wpiCFu8ThQ4pmS05ka";
//        VuforiaParameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK; //Sets phone to use back camera.
//        this.vuforia = ClassFactory.createVuforiaLocalizer(VuforiaParameters); //Initializes VuforiaLocalizer object as vuforia.
//    }

    //---UNIVERSAL METHODS BELOW---

    //Initializes Servos to a position.
    public void initServoPositions(){
        this.moveServo(this.jewelArm, this.JEWEL_ARM_UP, 750, 1000);
        this.wrist.setPosition(WRIST_FOLDED);
        this.elbow.setPosition(ELBOW_FOLDED);
        this.claw.setPosition(CLAW_CLOSED);
        this.clawTwist.setPosition(TWIST_UP);
}

    //Servo we want to move, Position we want to move to, Number of servo movements we want, the time we want this movement to occur over in milliseconds.
    public void moveServo(Servo targetServo, double targetPosition, int steps, long timeInMilli){
        //Total distance to travel.
        double distanceToTravel = Math.abs(targetServo.getPosition() - targetPosition);
        //Unit conversion to nanoseconds.
        long time = timeInMilli * 1000000;
        //Per Step values.
        //double distanceToTravelPerStep = (distanceToTravel / steps);
        long timePerStep = time / steps;
        //Loops number of steps.
        double distanceToTravelPerStep;
        if(targetPosition - targetServo.getPosition() >= 0){
            distanceToTravelPerStep = (distanceToTravel / steps);
        }else{
            distanceToTravelPerStep = (distanceToTravel / steps) * -1;
        }
        for(int counter = 0; counter < steps; counter++){
            double initialTime = System.nanoTime();
            double currentPosition = targetServo.getPosition(); //Gets current arm position.
            //if(movementFlag == 0) {
            targetServo.setPosition(currentPosition + distanceToTravelPerStep);//Moves the arm.
            while((System.nanoTime() - initialTime) < timePerStep){
                //Wait.
            }
        }
    }

    public static void moveMultipleServo(Servo targetServo1, Servo targetServo2, double targetPosition1, double targetPosition2, int steps, long timeInMilli){
        //Total distance to travel.
        double distanceToTravel1 = Math.abs(targetServo1.getPosition() - targetPosition1);
        double distanceToTravel2 = Math.abs(targetServo2.getPosition() - targetPosition2);
        //Unit conversion to nanoseconds.
        long time = timeInMilli * 1000000;
        //Per Step values.
        //double distanceToTravelPerStep = (distanceToTravel / steps);
        long timePerStep = time / steps;
        //Loops number of steps.
        double distanceToTravelPerStep1;
        double distanceToTravelPerStep2;

        if(targetPosition1 - targetServo1.getPosition() >= 0){
            distanceToTravelPerStep1 = (distanceToTravel1 / steps);
        }else{
            distanceToTravelPerStep1 = (distanceToTravel1 / steps) * -1;
        }

        if(targetPosition2 - targetServo2.getPosition() >= 0){
            distanceToTravelPerStep2 = (distanceToTravel2 / steps);
        }else{
            distanceToTravelPerStep2 = (distanceToTravel2 / steps) * -1;
        }

        for(int counter = 0; counter < steps; counter++){
            double initialTime = System.nanoTime();

            double currentPosition1 = targetServo1.getPosition(); //Gets current arm position.
            double currentPosition2 = targetServo2.getPosition(); //Gets current arm position.

            targetServo1.setPosition(currentPosition1 + distanceToTravelPerStep1);//Moves the arm.
            targetServo2.setPosition(currentPosition2 + distanceToTravelPerStep2);//Moves the arm.
            while((System.nanoTime() - initialTime) < timePerStep){
                //Wait.
            }
        }
    }

    //Drives at given power and a given distance unless floorSensors interrupt it by seeing the given color. ("red" or "blue").
    public void driveForwardWithInterrupt(double power, int distance, String color) {
        //Resets encoders by setting to STOP_AND_RESET_ENCODER mode.
        setRunMode("STOP_AND_RESET_ENCODER");
        //Sets target distance. Set to negative distance because motor was running backwards.
        setAllTargetPositions(-distance);
        //Sets to RUN_TO_POSITION mode
        setRunMode("RUN_TO_POSITION");
        //Sets power for DC Motors.
        setMotorPower(power);
        //Waits while driving to position.
        while(anyMotorsBusy()){
            if (color.equals("red")) {
                if(floorSensor.red() > 50) {//Level of Red required to stop.
                    setMotorPower(0.0);
                }
            }
            if(color.equals("blue")){
                if (floorSensor.blue() > 50) {//Level of Blue required to stop.
                    setMotorPower(0.0);
                }
            }
        }
        //Stops driving by setting power to 0.0.
        setMotorPower(0.0);
        //Sets back to RUN_USING_ENCODER mode.
        setRunMode("RUN_USING_ENCODER");
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
        while(anyMotorsBusy()){
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
        while(anyMotorsBusy()){
            //Spinning.
            //Waiting for robot to arrive at destination.
        }
        //Stops driving by setting power to 0.0.
        setMotorPower(0.0);
        //Sets back to RUN_USING_ENCODER mode.
        setRunMode("RUN_USING_ENCODER");
    }
    //It rotates the turret independant of encoders in the hardware class
//    public void rotateTurret(double power, int location, String direction){
//        //setRunMode("STOP_AND_RESET_ENCODER");
//        //setRunMode("RUN_TO_POSITION");
//        this.turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        if(direction.equals("CW")){
//            this.turretMotor.setTargetPosition(location);
//            this.turretMotor.setPower(-power);
//            while(this.turretMotor.isBusy()){
//                //waiting for turret to turn
//            }
//            this.turretMotor.setPower(0.0);
//            //setRunMode("RUN_USING_ENCODER");
//        }
//        if(direction.equals("CCW")){
//            this.turretMotor.setTargetPosition(location);
//            this.turretMotor.setPower(power);
//            while(this.turretMotor.isBusy()){
//                //waiting for turret to turn
//            }
//            this.turretMotor.setPower(0.0);
//            //setRunMode("RUN_USING_ENCODER");
//        }
//        this.turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//    }
    //Sets the run mode of all DC motors. Test is this works in both autonomous and teleOp modes.
    public void setRunMode(String input){
        if(input.equals("STOP_AND_RESET_ENCODER")) {
            this.left1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            this.left2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            this.right1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            this.right2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            //this.turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            //this.shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            //this.winchMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        if(input.equals("RUN_WITHOUT_ENCODER")) {
            this.left1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            this.left2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            this.right1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            this.right2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            //this.turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            //this.winchMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            //this.shoulder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        if(input.equals("RUN_USING_ENCODER")) {
            this.left1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            this.left2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            this.right1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            this.right2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //this.turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //this.winchMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //this.shoulder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        if(input.equals("RUN_TO_POSITION")) {
            this.left1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            this.left2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            this.right1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            this.right2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //this.turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //this.winchMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //this.shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
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
}