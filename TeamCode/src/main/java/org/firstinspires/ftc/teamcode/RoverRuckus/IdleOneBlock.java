package org.firstinspires.ftc.teamcode.RoverRuckus;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Func;
@Autonomous(name = "IdleOneBlock", group = "Autonomous")
public class IdleOneBlock extends LinearOpMode {
    public RoverHardware robot = new RoverHardware();

    private GoldAlignDetector detector;

    public void runOpMode() {
        robot.init(hardwareMap);

        detector = new GoldAlignDetector(); // Create detector
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance()); // Initialize it with the app context and camera
        detector.useDefaults(); // Set detector to use default settings

        // Optional tuning
        detector.alignSize = 100; // How wide (in pixels) is the range in which the gold object will be aligned. (Represented by green bars in the preview)
        detector.alignPosOffset = 0; // How far from center frame to offset this alignment zone.
        detector.downscale = 0.4; // How much to downscale the input frames

        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        detector.maxAreaScorer.weight = 0.005; //

        detector.ratioScorer.weight = 5; //
        detector.ratioScorer.perfectRatio = 1.0; // Ratio adjustment

        detector.enable(); // Start the detector!

        //Initialize Gyro
        BNO055IMU.Parameters parameters1 = new BNO055IMU.Parameters();
        parameters1.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters1.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters1.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters1.loggingEnabled = true;
        parameters1.loggingTag = "IMU";
        parameters1.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        robot.imu = hardwareMap.get(BNO055IMU.class, "imu");
        robot.imu.initialize(parameters1);

        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override
                    public String value() {
                        return robot.formatAngle(robot.angles.angleUnit, robot.angles.firstAngle);
                    }
                })
                .addData("roll", new Func<String>() {
                    @Override
                    public String value() {
                        return robot.formatAngle(robot.angles.angleUnit, robot.angles.secondAngle);
                    }
                })
                .addData("pitch", new Func<String>() {
                    @Override
                    public String value() {
                        return robot.formatAngle(robot.angles.angleUnit, robot.angles.thirdAngle);
                    }
                });
        waitForStart();


//        robot.hang.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.hang.setTargetPosition(1200);
//        robot.hang.setPower(0.8);
//        while (robot.hang.isBusy()){}

        //Raise arm
        while(robot.upperLimit.red() < 390){
            robot.hang.setPower(1);
        }
        robot.hang.setPower(0);
        sleep(500);

        //Drive forward slightly
        robot.left1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.right1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.left1.setTargetPosition(400);
        robot.right1.setTargetPosition(400);
        robot.left1.setPower(0.2);
        robot.right1.setPower(0.2);
        while(robot.left1.isBusy()){}


        //Hunt for the Block
        while (detector.getXPosition() < 235 || detector.getXPosition() > 345){
            telemetry.addData("Status", "searching for angle");
            telemetry.addData("xpos", detector.getXPosition());
            telemetry.addData("IsAligned", detector.getAligned());
            if (detector.getXPosition() < 235) {
                robot.left1.setPower(.3);
                robot.right1.setPower(-.3);
            } else if (detector.getXPosition() > 340) {
                robot.left1.setPower(-.3);
                robot.right1.setPower(.3);
            }
        }
        robot.left1.setPower(0);
        robot.right1.setPower(0);
        sleep(500);

        //Lower intake and extend arm out
        robot.bop.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.bop.setTargetPosition(1200);
        robot.bop.setPower(0.4);
        while(robot.bop.isBusy()) {
            robot.drop.setPosition(robot.BOTTOM_INTAKE);
        }
        //turn right
        while(robot.angles.firstAngle > 93 || robot.angles.firstAngle < 87){
            if(robot.angles.firstAngle > 93){
                robot.left1.setPower(-0.3);
                robot.right1.setPower(0.3);
            } else if(robot.angles.firstAngle < 87){
                robot.left1.setPower(0.3);
                robot.right1.setPower(-0.3);
            }
        }

        //Drive to wall
        robot.setRunMode("STOP_AND_RESET_ENCODERS");
        robot.setRunMode("RUN_TO_POSITION");
        robot.setAllTargetPositions(-1200);
        robot.setMotorPower(-0.2);

        //turn parallel to wall
        while(robot.angles.firstAngle > 0 || (robot.angles.firstAngle > -170 && robot.angles.firstAngle < 0)){ //target -170
            robot.left1.setPower(-0.3);
            robot.right1.setPower(0.3);
        }

        //Drive forward
        robot.setRunMode("STOP_AND_RESET_ENCODERS");
        robot.setRunMode("RUN_TO_POSITION");
        robot.setAllTargetPositions(-1000);
        robot.setMotorPower(-0.2);

        //Drop off icon

        //turn to opposite crater
        while(robot.angles.firstAngle > 0 || (robot.angles.firstAngle > -90 && robot.angles.firstAngle < 0)){ //target -170
            robot.left1.setPower(-0.3);
            robot.right1.setPower(0.3);
        }

        //Drive forward to crater
        robot.setRunMode("STOP_AND_RESET_ENCODERS");
        robot.setRunMode("RUN_TO_POSITION");
        robot.setAllTargetPositions(-2600);
        robot.setMotorPower(-0.2);
    }
}