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

@Autonomous (name = "CraterTwoBlock", group = "Autonomous")
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

        //Raise arm
        while (robot.upperLimit.red() > 300 &&opModeIsActive()) {
            robot.hang.setPower(1);
        }
        robot.hang.setPower(0);
        sleep(500);

        //Drive forward slightly
        robot.left1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.right1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.left1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.right1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.left1.setTargetPosition(900);
        robot.right1.setTargetPosition(900);
        robot.left1.setPower(0.2);
        robot.right1.setPower(0.2);
        while (robot.left1.isBusy() && robot.right1.isBusy() && opModeIsActive()) {
            telemetry.addData("right power", robot.right1.getPower());
            telemetry.addData("right position", robot.right1.getCurrentPosition());
            telemetry.addData("left power", robot.left1.getPower());
            telemetry.addData("left position", robot.left1.getCurrentPosition());
            telemetry.update();

            if(opModeIsActive() == false){
                break;
            }
        }
        //Hunt for the Block
        robot.left1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.right1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        while(detector.getXPosition() < 235 && opModeIsActive() || detector.getXPosition() > 345 &&opModeIsActive()){
            telemetry.addData("Status", "searching for angle");
            telemetry.addData("xpos", detector.getXPosition());
            telemetry.addData("IsAligned", detector.getAligned());
            if (detector.getXPosition() < 235){
                robot.left1.setPower(.4);
                robot.right1.setPower(-.4);
            } else if (detector.getXPosition() > 340){
                robot.left1.setPower(-.4);
                robot.right1.setPower(.4);
            }


            if(opModeIsActive() == false){
                break;
            }
        }
        robot.left1.setPower(0);
        robot.right1.setPower(0);
        sleep(500);

        //Change the arm angle so it can hit the block
        robot.rotateMech.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rotateMech.setTargetPosition(180);
        robot.rotateMech.setPower(0.1);
        while(robot.rotateMech.isBusy()){}

//        //Lower intake and extend arm out
        robot.bop.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.bop.setTargetPosition(-1800);
        robot.bop.setPower(-0.4);
        while(robot.bop.isBusy() && opModeIsActive()) {
            robot.drop.setPosition(robot.BOTTOM_INTAKE);
        }

        //bring arm back in
//        robot.bop.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.bop.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.bop.setTargetPosition(-50);
        robot.bop.setPower(0.4);
        while(robot.bop.isBusy() && opModeIsActive() && opModeIsActive()){
            robot.drop.setPosition(robot.TOP_INTAKE);
        }

//        //turn right
        telemetry.update();
        while (robot.angles.firstAngle > 75 && opModeIsActive() || robot.angles.firstAngle < 70 && opModeIsActive()) {
            telemetry.update();
            if (robot.angles.firstAngle > 75) {
                robot.left1.setPower(-0.5);
                robot.right1.setPower(0.5);
                telemetry.update();
            } else if (robot.angles.firstAngle < 70) {
                robot.left1.setPower(0.5);
                robot.right1.setPower(-0.5);
            }
            telemetry.update();
        }
////
        //Drive to the team marker area
        robot.right1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.left1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.right1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.left1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.left1.setTargetPosition(7300);
        robot.right1.setTargetPosition(7300);
        robot.left1.setPower(0.4);
        robot.right1.setPower(0.4);
        while (robot.left1.isBusy() && opModeIsActive() || robot.right1.isBusy() && opModeIsActive()){}
        robot.right1.setPower(0);
        robot.left1.setPower(0);
//
        //Turn Parallel with wall
        telemetry.update();
        robot.left1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.right1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        while (robot.angles.firstAngle > -55 && opModeIsActive() || (robot.angles.firstAngle < -65 && robot.angles.firstAngle < 0)&& opModeIsActive()) {
            robot.left1.setPower(-0.5);
            robot.right1.setPower(0.5);
            telemetry.update();
        }
        robot.left1.setPower(0);
        robot.right1.setPower(0);

        //Drive forward and slightly into the wall
        robot.left1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.right1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.left1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//////        robot.right1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//////        telemetry.addData("Color Sensor RED", robot.cornerSensor.red());
//////        telemetry.addData("Color Sensor BLUE", robot.cornerSensor.blue());
//////        telemetry.update();
//////        while(robot.cornerSensor.red() < 50 && opModeIsActive()|| robot.cornerSensor.blue() < 15 &&opModeIsActive()){
//////            robot.left1.setPower(-0.4 * 1.03);
//////            robot.right1.setPower(-0.4);
//////            telemetry.addData("Color Sensor RED", robot.cornerSensor.red());
//////            telemetry.addData("Color Sensor BLUE", robot.cornerSensor.blue());
//////            telemetry.update();
//////        }
        robot.left1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.right1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.left1.setTargetPosition(-5000);
        robot.right1.setTargetPosition(-5000);
        robot.left1.setPower(-0.4 *  1.05);
        robot.right1.setPower(-0.4 );
        while(robot.left1.isBusy()){}
        robot.left1.setPower(0);
        robot.right1.setPower(0);

        //Turn and prep to drop off marker
        telemetry.update();
        robot.left1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.right1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        while (robot.angles.firstAngle > 5 && opModeIsActive() || (robot.angles.firstAngle < -5 && robot.angles.firstAngle < 0)&& opModeIsActive()) {
            robot.left1.setPower(0.5);
            robot.right1.setPower(-0.5);
            telemetry.update();
        }
        robot.left1.setPower(0);
        robot.right1.setPower(0);
//
        telemetry.update();
        robot.left1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.right1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        while (robot.angles.firstAngle > -35 && opModeIsActive() || (robot.angles.firstAngle < -45 && robot.angles.firstAngle < 0)&& opModeIsActive()) {
            robot.left1.setPower(-0.5);
            robot.right1.setPower(0.5);
            telemetry.update();
        }
        robot.left1.setPower(0);
        robot.right1.setPower(0);
//
//
//            //Drop Icon thing
//            //robot.drop.setPosition(0.2);

        //turn the rest of the way to the blocks
        telemetry.update();
        robot.left1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.right1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        while (robot.angles.firstAngle > -65 && opModeIsActive() || (robot.angles.firstAngle < -75 && robot.angles.firstAngle < 0)&& opModeIsActive()) {
            robot.left1.setPower(-0.5);
            robot.right1.setPower(0.5);
            telemetry.update();
        }
        robot.left1.setPower(0);
        robot.right1.setPower(0);

        //Aim and pick the block
        robot.left1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.right1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        while(detector.getXPosition() < 235 && opModeIsActive() || detector.getXPosition() > 345 &&opModeIsActive()) {
            telemetry.addData("Status", "searching for angle");
            telemetry.addData("xpos", detector.getXPosition());
            telemetry.addData("IsAligned", detector.getAligned());
            if (detector.getXPosition() < 235) {
                robot.left1.setPower(.4);
                robot.right1.setPower(-.4);
            } else if (detector.getXPosition() > 340) {
                robot.left1.setPower(-.4);
                robot.right1.setPower(.4);
            }
        }

        //Hit the block
        robot.bop.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.bop.setTargetPosition(-1800);
        robot.bop.setPower(-0.4);
        while(robot.bop.isBusy() && opModeIsActive()) {
            robot.drop.setPosition(robot.BOTTOM_INTAKE);
        }

        //bring arm back in
        robot.bop.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.bop.setTargetPosition(-50);
        robot.bop.setPower(0.4);
        while(robot.bop.isBusy() && opModeIsActive() && opModeIsActive()){
            robot.drop.setPosition(robot.TOP_INTAKE);
        }

        //Turn to other wall
        telemetry.update();
        robot.left1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.right1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        while (robot.angles.firstAngle > 45 && opModeIsActive() || robot.angles.firstAngle < 35 && opModeIsActive()){
            robot.left1.setPower(0.5);
            robot.right1.setPower(-0.5);
            telemetry.update();
        }
        robot.left1.setPower(0);
        robot.right1.setPower(0);

        //Ride the wall to the other team's crater
        robot.left1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.right1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.left1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.right1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.left1.setTargetPosition(3500);
        robot.right1.setTargetPosition(3500);
        robot.left1.setPower(0.5 * 1.05);
        robot.right1.setPower(0.5);
        while (robot.left1.isBusy() || robot.right1.isBusy()) {}
    }
}





