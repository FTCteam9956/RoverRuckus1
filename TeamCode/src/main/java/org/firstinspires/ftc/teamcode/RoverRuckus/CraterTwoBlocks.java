package org.firstinspires.ftc.teamcode.RoverRuckus;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Func;

/**
 * Created by salty on 11/17/2018.
 */

public class CraterTwoBlocks extends LinearOpMode {
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

        robot.hang.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.hang.setTargetPosition(1200);
        robot.hang.setPower(0.8);
        while (robot.hang.isBusy()){}

        //Hunt for the Block
        while (detector.getXPosition() < 235 || detector.getXPosition() > 345) {
            telemetry.addData("Status", "searching for angle");
            telemetry.addData("xpos", detector.getXPosition());
            telemetry.addData("IsAligned", detector.getAligned());
            if (detector.getXPosition() < 235) {
                robot.left1.setPower(.05);
                robot.right1.setPower(-.05);
            } else if (detector.getXPosition() > 340) {
                robot.left1.setPower(-.05);
                robot.right1.setPower(.05);
            }
        }

        //Drive Towards the Block
        robot.left1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.right1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.left1.setTargetPosition(1000);
        robot.right1.setTargetPosition(1000);
        robot.left1.setPower(0.1);
        robot.right1.setPower(0.1);
        while (robot.left1.isBusy() || robot.right1.isBusy()) {
        }

        //Backup so we don't hit other blocks
        robot.left1.setTargetPosition(-500);
        robot.right1.setTargetPosition(-500);
        robot.left1.setPower(-0.1);
        robot.right1.setPower(-0.1);
        while (robot.left1.isBusy() || robot.right1.isBusy()) {
        }

        //turn right
        while (robot.angles.firstAngle > 65 || robot.angles.firstAngle < 55) {
            if (robot.angles.firstAngle > 65) {
                robot.left1.setPower(-0.1);
                robot.right1.setPower(0.1);
            } else if (robot.angles.firstAngle < 55) {
                robot.left1.setPower(0.1);
                robot.right1.setPower(-0.1);
            }

            robot.left1.setTargetPosition(800);
            robot.right1.setTargetPosition(800);
            robot.left1.setPower(0.1);
            robot.right1.setPower(0.1);
            while (robot.left1.isBusy() || robot.right1.isBusy()) {}

            //Turn Parallel with wall
            while (robot.angles.firstAngle > 95 || robot.angles.firstAngle > 85){
                if (robot.angles.firstAngle > 95) {
                    robot.left1.setPower(0.1);
                    robot.right1.setPower(-0.1);
                } else if (robot.angles.firstAngle < 85){
                    robot.left1.setPower(-0.1);
                    robot.right1.setPower(0.1);
                }
            }

            //Drive behind blocks
            robot.left1.setTargetPosition(1000);
            robot.right1.setTargetPosition(1000);
            robot.left1.setPower(0.1);
            robot.right1.setPower(0.1);
            while (robot.left1.isBusy() || robot.right1.isBusy()) {}

            //Turn Towards the Blocks
            while (robot.angles.firstAngle > 10 || robot.angles.firstAngle > -10){
                if (robot.angles.firstAngle > 10) {
                    robot.left1.setPower(0.1);
                    robot.right1.setPower(-0.1);
                } else if (robot.angles.firstAngle < -10){
                    robot.left1.setPower(-0.1);
                    robot.right1.setPower(0.1);
                }
            }

            //Aim to the block
            while (detector.getXPosition() < 235 || detector.getXPosition() > 345) {
                telemetry.addData("Status", "searching for angle");
                telemetry.addData("xpos", detector.getXPosition());
                telemetry.addData("IsAligned", detector.getAligned());
                if (detector.getXPosition() < 235) {
                    robot.left1.setPower(.05);
                    robot.right1.setPower(-.05);
                } else if (detector.getXPosition() > 340) {
                    robot.left1.setPower(-.05);
                    robot.right1.setPower(.05);
                }
            }

            //Hit block
            robot.left1.setTargetPosition(800);
            robot.right1.setTargetPosition(800);
            robot.left1.setPower(0.1);
            robot.right1.setPower(0.1);
            while (robot.left1.isBusy() || robot.right1.isBusy()) {}

            //Drop Icon thing
            //robot.drop.setPosition(0.2);

            //Turn Torwards Crater
            while (robot.angles.firstAngle > 30 || robot.angles.firstAngle < 40){
                if (robot.angles.firstAngle > 30) {
                    robot.left1.setPower(0.1);
                    robot.right1.setPower(-0.1);
                } else if (robot.angles.firstAngle < 40){
                    robot.left1.setPower(-0.1);
                    robot.right1.setPower(0.1);
                }
            }

            //Backup into crater
            robot.left1.setTargetPosition(-2600);
            robot.right1.setTargetPosition(-2600);
            robot.left1.setPower(0.1);
            robot.right1.setPower(0.1);
            while (robot.left1.isBusy() || robot.right1.isBusy()){}
        }
    }
}
