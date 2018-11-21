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
//import com.qualcomm.robotcore.hardware.HardwareMap;
//
//import org.firstinspires.ftc.robotcore.external.Func;
//@Autonomous(name = "OneBlockCrater", group = "Autonomous")
////@Disabled
//public class BlueShort extends LinearOpMode {
//    public RoverHardware robot = new RoverHardware();
//
//    private GoldAlignDetector detector;
//
//    public void runOpMode(){
//        robot.init(hardwareMap);
//
//        robot.left1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        //robot.left2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.right1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        //robot.right2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        //Initialize OpenCV
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
//
//        //Initialize Gyro
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
//        telemetry.addData("xpos", detector.getXPosition());
//
//        //Lower off of the Lander
////        robot.hang.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////        robot.hang.setTargetPosition(1500);
////        robot.hang.setPower(0.8);
////        while(robot.hang.isBusy()){}
//
//        //Hunt for the Block
//        while(detector.getXPosition() < 235 || detector.getXPosition() > 345){
//            telemetry.addData("Status","searching for angle");
//            telemetry.addData("xpos", detector.getXPosition());
//            telemetry.addData("IsAligned" , detector.getAligned());
//            if (detector.getXPosition() < 235){
//                robot.left1.setPower(.05);
//                robot.right1.setPower(-.05);
//                //robot.left2.setPower(.05);
//                //robot.right2.setPower(-.05);
//            }
//            else if (detector.getXPosition() > 340){
//                robot.left1.setPower(-.05);
//                robot.right1.setPower(.05);
//                //robot.left2.setPower(-.05);
//                //robot.right2.setPower(.05);
//            }
//        }
//
//        //Drive Forward torwards the block
//        robot.left1.setTargetPosition(750);
//        //robot.left2.setTargetPosition(750);
//        robot.right1.setTargetPosition(750);
//        //robot.right2.setTargetPosition(750);
//        robot.left1.setPower(.2);
//        //robot.left2.setPower(.2);
//        robot.right1.setPower(.2);
//        //robot.right2.setPower(.2);
//
//        robot.left1.setTargetPosition(-500);
//        //robot.left2.setTargetPosition(-500);
//        robot.right1.setTargetPosition(-500);
//        //robot.right2.setTargetPosition(-500);
//        robot.left1.setPower(-.2);
//        //robot.left2.setPower(-.2);
//        robot.right1.setPower(-.2);
//        //robot.right2.setPower(-.2);
//        while(robot.anyMotorsBusy()){}
//
//        while(robot.angles.firstAngle < 85){
//            robot.left1.setPower(-.2);
//            //robot.left2.setPower(-.2);
//            robot.right1.setPower(.2);
//            //robot.right2.setPower(.2);
//        }
//
//        robot.left1.setTargetPosition(700);
//        //robot.left2.setTargetPosition(700);
//        robot.right1.setTargetPosition(700);
//        //robot.right2.setTargetPosition(700);
//        robot.left1.setPower(.2);
//        //robot.left2.setPower(.2);
//        robot.right1.setPower(.2);
//        //robot.right2.setPower(.2);
//
//        while(robot.angles.firstAngle < 150){
//            robot.left1.setPower(-.2);
//            //robot.left2.setPower(-.2);
//            robot.right1.setPower(.2);
//            //robot.right2.setPower(.2);
//        }
//
//        robot.left1.setTargetPosition(1300);
//        //robot.left2.setTargetPosition(1300);
//        robot.right1.setTargetPosition(1300);
//        //robot.right2.setTargetPosition(1300);
//        robot.left1.setPower(.2 *1.03);
//        //robot.left2.setPower(.2 * 1.03);
//        robot.right1.setPower(.2);
//        //robot.right2.setPower(.2);
//        while(robot.anyMotorsBusy()){}
//
//        robot.left1.setTargetPosition(-2000);
//        robot.right1.setTargetPosition(-2000);
//        robot.left1.setPower(0.3);
//        robot.right1.setPower(0.3);
//        while(robot.anyMotorsBusy()){}
//    }
//}