//package org.firstinspires.ftc.teamcode.RoverRuckus;
//
//import com.disnodeteam.dogecv.CameraViewDisplay;
//import com.disnodeteam.dogecv.DogeCV;
//import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
//import com.qualcomm.hardware.bosch.BNO055IMU;
//import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//
//@Autonomous(name= "GoldScoutAuto", group= "Autonomous")
//
//public class HitBlockTest extends LinearOpMode{
//    public RoverHardware robot = new RoverHardware();
//    public DcMotor left1;
//    public DcMotor left2;
//    public DcMotor right1;
//    public DcMotor right2;
//
//    private GoldAlignDetector detector;
//
//    public void runOpMode(){
//        robot.init(hardwareMap);
//        left1 = hardwareMap.dcMotor.get("left1");
//        left2 = hardwareMap.dcMotor.get("left2");
//        right1 = hardwareMap.dcMotor.get("right1");
//        right2 = hardwareMap.dcMotor.get("right2");
//
//        robot.left1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.left2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.right1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.right2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        left1.setDirection(DcMotorSimple.Direction.REVERSE);
//        left2.setDirection(DcMotorSimple.Direction.REVERSE);
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
//        waitForStart();
//
//        telemetry.addData("xpos", detector.getXPosition());
//        robot.hang.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.hang.setTargetPosition(2000);
//        robot.hang.setPower(.8);
//
//        sleep(100000);
//
//        while(detector.getXPosition() < 235 || detector.getXPosition() > 345){
//            telemetry.addData("Status","searching for angle");
//            telemetry.addData("xpos", detector.getXPosition());
//            telemetry.addData("IsAligned" , detector.getAligned());
//            if (detector.getXPosition() < 235){
//                robot.left1.setPower(.05);
//                robot.right1.setPower(-.05);
//                robot.left2.setPower(.05);
//                robot.right2.setPower(-.05);
//            }
//            else if (detector.getXPosition() > 340){
//                robot.left1.setPower(-.05);
//                robot.right1.setPower(.05);
//                robot.left2.setPower(-.05);
//                robot.right2.setPower(.05);
//            }
//        }
//        robot.left1.setTargetPosition(750);
//        robot.left2.setTargetPosition(750);
//        robot.right1.setTargetPosition(750);
//        robot.right2.setTargetPosition(750);
//        robot.left1.setPower(.2);
//        robot.left2.setPower(.2);
//        robot.right1.setPower(.2);
//        robot.right2.setPower(.2);
//        sleep(3000);
//
//    }
//
//}
