//package org.firstinspires.ftc.teamcode.RoverRuckus;
//
//import com.disnodeteam.dogecv.CameraViewDisplay;
//import com.disnodeteam.dogecv.DogeCV;
//import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
//import com.qualcomm.hardware.bosch.BNO055IMU;
//import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//
//import org.firstinspires.ftc.robotcore.external.Func;
//
//@Autonomous(name = "1BlockOppositePark",group = "Autonomous")
////@Disabled
//public class Auti extends LinearOpMode {
//    public RoverHardware robot = new RoverHardware();
//    private GoldAlignDetector detector;
//    public void runOpMode() {
//        robot.init(hardwareMap);
//        detector = new GoldAlignDetector();
//        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
//        detector.useDefaults();
//
//        // Optional Tuning
//        detector.alignSize = 100; // How wide (in pixels) is the range in which the gold object will be aligned. (Represented by green bars in the preview)
//        detector.alignPosOffset = 0; // How far from center frame to offset this alignment zone.
//        detector.downscale = 0.4; // How much to downscale the input frames
//
//        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
//        detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
//        detector.maxAreaScorer.weight = 0.005;
//
//        detector.ratioScorer.weight = 5;
//        detector.ratioScorer.perfectRatio = 1.0;
//
//        detector.enable();
//        BNO055IMU.Parameters parameters1 = new BNO055IMU.Parameters();
//        parameters1.angleUnit = BNO055IMU.AngleUnit.DEGREES;
//        parameters1.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
//        parameters1.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
//        parameters1.loggingEnabled = true;
//        parameters1.loggingTag = "IMU";
//        parameters1.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
//
//        robot.imu = hardwareMap.get(BNO055IMU.class, "imu");
//        robot.imu.initialize(parameters1);
//
//        telemetry.addLine()
//                .addData("heading", new Func<String>() {
//                    @Override
//                    public String value() {
//                        return robot.formatAngle(robot.angles.angleUnit, robot.angles.firstAngle);
//                    }
//                })
//                .addData("roll", new Func<String>() {
//                    @Override
//                    public String value() {
//                        return robot.formatAngle(robot.angles.angleUnit, robot.angles.secondAngle);
//                    }
//                })
//                .addData("pitch", new Func<String>() {
//                    @Override
//                    public String value() {
//                        return robot.formatAngle(robot.angles.angleUnit, robot.angles.thirdAngle);
//                    }
//                });
//        waitForStart();
//
//        telemetry.addData("xposition", detector.getXPosition());
//
//        //Lower robot
////        robot.hang.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////        robot.hang.setTargetPosition(2000);
////        robot.hang.setPower(.8);
////        while (robot.hang.isBusy()) {}
//
////        while (detector.getXPosition() > 345 || detector.getXPosition() < 245) {
////            telemetry.addData("Status", "searching for angle");
////            telemetry.addData("xpos", detector.getXPosition());
////            telemetry.addData("IsAligned", detector.getAligned());
////            if (detector.getXPosition() > 345) {
////                robot.left1.setPower(-.2);
////                robot.right1.setPower(.2);
////            } else if (detector.getXPosition() < 245) {
////                robot.right1.setPower(-.2);
////                robot.left1.setPower(.2);
////            }
////            telemetry.addData("status", "Hunting for Block");
////        }
////        robot.right1.setPower(0);
////        robot.left1.setPower(0);
////        sleep(500);
//
//        //driveforward
//        robot.left1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.right1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.left1.setTargetPosition(1200);
//        robot.right1.setTargetPosition(1200);
//        robot.left1.setPower(.2);
//        robot.right1.setPower(.2);
//        while(robot.left1.isBusy() || robot.right1.isBusy()) {
//            telemetry.addData("Status", "Driving forward");
//        }
//
//        robot.left1.setPower(0);
//        robot.right1.setPower(0);
//        sleep(1000);
//
////        //bopping the block
//////        robot.bop.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//////        robot.bop.setTargetPosition(-1200);
//////        robot.bop.setPower(-.1);
//////        while (robot.bop.isBusy()) {}
////
////        //turnleft
////        while (robot.angles.firstAngle < 47 && robot.angles.firstAngle > 53) {
////            if (robot.angles.firstAngle < 47) {
////                robot.left1.setPower(.1);
////                robot.right1.setPower(-.1);
////            } else if (robot.angles.firstAngle > 53) {
////                robot.left1.setPower(-.1);
////                robot.right1.setPower(.1);
////            }
////        }
////
////        //driveforward
////        robot.left1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////        robot.right1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////        robot.left1.setTargetPosition(5000);
////        robot.right1.setTargetPosition(5000);
////        robot.left1.setPower(.2);
////        robot.right1.setPower(.2);
////        while(robot.left1.isBusy() || robot.right1.isBusy()) {}
////
////        //wallturn
////        while (robot.angles.firstAngle < 17 && robot.angles.firstAngle > 23) {
////            if (robot.angles.firstAngle < 17) {
////                robot.left1.setPower(.1);
////                robot.right1.setPower(-.1);
////            } else if (robot.angles.firstAngle > 23) {
////                robot.left1.setPower(-.1);
////                robot.right1.setPower(.1);
////            }
////        }
////
////        robot.left1.setTargetPosition(1200);
////        robot.right1.setTargetPosition(1200);
////        robot.left1.setPower(.2 * 1.03);
////        robot.right1.setPower(.2);
////        while(robot.left1.isBusy() || robot.right1.isBusy()) {}
////
////        //droppe
////        robot.drop.setPosition(.5);
////
////        //reverse (set mode?/negative left)
////
////        robot.left1.setTargetPosition(-1200);
////        robot.right1.setTargetPosition(-1200);
////        robot.left1.setPower(-.2 * 1.03);
////        robot.right1.setPower(-.2);
////        while(robot.left1.isBusy() || robot.right1.isBusy()) {}
////
////        //
//
//    }
//}
