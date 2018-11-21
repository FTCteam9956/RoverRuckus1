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

@Autonomous(name = "CraterOneBlock", group = "Autonomous")
//@Disabled
public class CraterOneBlock extends LinearOpMode {
    public RoverHardware robot = new RoverHardware();

    private GoldAlignDetector detector;

    public void runOpMode(){
        robot.init(hardwareMap);

        //Initialize OpenCV
        detector = new GoldAlignDetector();
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        detector.useDefaults();

        // Optional Tuning
        detector.alignSize = 100; // How wide (in pixels) is the range in which the gold object will be aligned. (Represented by green bars in the preview)
        detector.alignPosOffset = 0; // How far from center frame to offset this alignment zone.
        detector.downscale = 0.4; // How much to downscale the input frames

        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        detector.maxAreaScorer.weight = 0.005;

        detector.ratioScorer.weight = 5;
        detector.ratioScorer.perfectRatio = 1.0;

        detector.enable();

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
        while(robot.bop.isBusy()){
            robot.drop.setPosition(robot.BOTTOM_INTAKE);
        }

//        //turn right
//        while (robot.angles.firstAngle > 65 || robot.angles.firstAngle < 55) {
//            if (robot.angles.firstAngle > 65) {
//                robot.left1.setPower(-0.1);
//                robot.right1.setPower(0.1);
//            } else if (robot.angles.firstAngle < 55) {
//                robot.left1.setPower(0.1);
//                robot.right1.setPower(-0.1);
//            }
//
//            robot.left1.setTargetPosition(800);
//            robot.right1.setTargetPosition(800);
//            robot.left1.setPower(0.1);
//            robot.right1.setPower(0.1);
//            while (robot.left1.isBusy() || robot.right1.isBusy()) {}
//
//            //Turn Parallel with wall
//            while (robot.angles.firstAngle > 75 || robot.angles.firstAngle < 65) {
//                if (robot.angles.firstAngle > 75) {
//                    robot.left1.setPower(-0.1);
//                    robot.right1.setPower(0.1);
//                } else if (robot.angles.firstAngle < 75) {
//                    robot.left1.setPower(0.1);
//                    robot.right1.setPower(-0.1);
//                }
//            }
//
//            //Drive forward and slightly into the wall
//            robot.left1.setTargetPosition(800);
//            robot.right1.setTargetPosition(800);
//            robot.left1.setPower(0.1 * 1.03);
//            robot.right1.setPower(0.1);
//            while (robot.left1.isBusy() || robot.right1.isBusy()) {}
//
//            //Drop Icon thing
//            //robot.drop.setPosition(0.2);
//
//            //Backup into crater
//            robot.left1.setTargetPosition(-2600);
//            robot.right1.setTargetPosition(-2600);
//            robot.left1.setPower(0.1);
//            robot.right1.setPower(0.1);
//            while (robot.left1.isBusy() || robot.right1.isBusy()) {}
//        }
    }
}