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
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.openftc.revextensions2.RevExtensions2;

import java.util.Locale;

@Autonomous(name = "BlueDepotOneBlock", group = "Autonomous")
public class IdleOneBlock extends LinearOpMode {
    public RoverHardware robot = new RoverHardware();

    private GoldAlignDetector detector;

    float angleTurn;
    int blue;

    public void runOpMode() {robot.init(hardwareMap);

        RevExtensions2.init();

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

        composeTelemetry();

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
        while(!opModeIsActive()){
            telemetry.update();
        }
        waitForStart();

        //Raise arm
        while (robot.upperLimit.red() > 300 && opModeIsActive()) {
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

            if (opModeIsActive() == false) {
                break;
            }
        }

        if (detector.getAligned() == true || detector.getAligned() == false) {
            //Hunt for the Block
            robot.left1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.right1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            while (detector.getXPosition() < 235 && opModeIsActive() || detector.getXPosition() > 345 && opModeIsActive()) {
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
        } else {
            robot.angles.firstAngle = angleTurn;

            robot.left1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.right1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            while (robot.angles.firstAngle > 14 && opModeIsActive() || robot.angles.firstAngle < 19 && opModeIsActive()) {
                angleTurn = robot.angles.firstAngle;
                //This is a right turn to 78 degrees
                robot.left1.setPower(Math.abs((17 - angleTurn) / 17) * 0.4);
                robot.right1.setPower(Math.abs((17 - angleTurn) / 17) * -0.4);
                telemetry.addData("left1 power", robot.left1.getPower());
                telemetry.addData("right1 power", robot.right1.getPower());
                telemetry.addData("heading", robot.angles.firstAngle);
                telemetry.addData("angle var:", angleTurn);
                telemetry.update();

            }
        }
        robot.left1.setPower(0);
        robot.right1.setPower(0);
        sleep(500);

        robot.drop.setPosition(robot.BOTTOM_INTAKE);
        sleep(500);

        //Change the arm angle so it can hit the block
        robot.rotateMech.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rotateMech.setTargetPosition(180);
        robot.rotateMech.setPower(0.1);
        while (robot.rotateMech.isBusy()) {
        }

//        //Lower intake and extend arm out
        telemetry.addData("heading", robot.angles.firstAngle);
        telemetry.update();
        if(robot.angles.firstAngle < 5 && robot.angles.firstAngle > -5) {
            robot.bop.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.bop.setTargetPosition(-1050);
            robot.bop.setPower(-0.8);
            while (robot.bop.isBusy() && opModeIsActive()) {
            }


        } else{
            robot.bop.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.bop.setTargetPosition(-1200);
            robot.bop.setPower(-0.4);
            while (robot.bop.isBusy() && opModeIsActive()) {
            }

        }

        //bring arm back in
        robot.bop.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.bop.setTargetPosition(750);
        robot.bop.setPower(0.4);
        while (robot.bop.isBusy() && opModeIsActive() && opModeIsActive()) {

        }
        robot.drop.setPosition(robot.TOP_INTAKE);

        //turn right
        telemetry.update();
        robot.left1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.right1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        while (robot.angles.firstAngle > -77 && opModeIsActive() || (robot.angles.firstAngle < -90 && robot.angles.firstAngle < 0) && opModeIsActive()) {
            robot.left1.setPower(Math.abs((-90 - robot.angles.firstAngle) / -50) * -0.7);
            robot.right1.setPower(Math.abs((-90 - robot.angles.firstAngle) / -50) * 0.7);
            telemetry.addData("left1 power", robot.left1.getPower());
            telemetry.addData("right1 power", robot.right1.getPower());
            telemetry.addData("heading", robot.angles.firstAngle);
            telemetry.addData("angle var:", angleTurn);
            telemetry.update();
        }
        robot.left1.setPower(0);
        robot.right1.setPower(0);

        //Drive to other blocks
        robot.right1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.left1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.right1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.left1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.left1.setTargetPosition(5000);
        robot.right1.setTargetPosition(5000);
        robot.left1.setPower(0.4);
        robot.right1.setPower(0.4);
        while (robot.left1.isBusy() && opModeIsActive() || robot.right1.isBusy() && opModeIsActive()) {}
        robot.right1.setPower(0);
        robot.left1.setPower(0);

        //turn right
        telemetry.update();
        robot.left1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.right1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        while (robot.angles.firstAngle > -165 && opModeIsActive() || (robot.angles.firstAngle < -185 && robot.angles.firstAngle < 0) && opModeIsActive()) {
            robot.left1.setPower(Math.abs((-170 - robot.angles.firstAngle) / -45) * -0.8);
            robot.right1.setPower(Math.abs((-170 - robot.angles.firstAngle) / -45) * 0.8);
            telemetry.addData("left1 power", robot.left1.getPower());
            telemetry.addData("right1 power", robot.right1.getPower());
            telemetry.addData("heading", robot.angles.firstAngle);
            telemetry.addData("angle var:", angleTurn);
            telemetry.update();
        }
        robot.left1.setPower(0);
        robot.right1.setPower(0);
//
        robot.left1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.right1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.left1.setTargetPosition(-2500);
        robot.right1.setTargetPosition(-2500);
        robot.left1.setPower(-0.6 * 1.1);
        robot.right1.setPower(-0.6);
        while (robot.left1.isBusy()) {}
        robot.left1.setPower(-0.5 * 1.1);
        robot.right1.setPower(-0.5);
        robot.left1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.right1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        telemetry.addData("Color Sensor BLUE", robot.cornerSensor.blue());
        telemetry.update();

        while(robot.cornerSensor.blue() <= 45 &&opModeIsActive()){
            blue = robot.cornerSensor.blue();

            robot.left1.setPower(-0.3 * 1.03);
            robot.right1.setPower(-0.3);
            telemetry.addData("Color Sensor BLUE", robot.cornerSensor.blue());
            telemetry.addData("Alpha", robot.cornerSensor.alpha());
            telemetry.addData("BLUE", blue);
            telemetry.addData("Searching", "");
            telemetry.update();
        }
        robot.left1.setPower(0);
        robot.right1.setPower(0);

        //Turn and prep to drop off marker
        telemetry.update();
        robot.left1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.right1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        while (robot.angles.firstAngle > -100 && opModeIsActive() || (robot.angles.firstAngle < -115 && robot.angles.firstAngle < 0) && opModeIsActive()) {
            robot.left1.setPower(0.4);
            robot.right1.setPower(-0.4);
            telemetry.update();
        }
        robot.left1.setPower(0);
        robot.right1.setPower(0);

        robot.marker.setPosition(robot.DILBERT_DOWN);
        sleep(500);

        //Turn and prep to drop off marker
        telemetry.update();
        robot.left1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.right1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        while (robot.angles.firstAngle > -115 && opModeIsActive() || (robot.angles.firstAngle < -125 && robot.angles.firstAngle < 0) && opModeIsActive()) {
            robot.left1.setPower(0.4);
            robot.right1.setPower(-0.4);
            telemetry.update();
        }
        robot.left1.setPower(0);
        robot.right1.setPower(0);

//        robot.left1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.right1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.left1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.right1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.left1.setTargetPosition(6000);
//        robot.right1.setTargetPosition(6000);
//        robot.left1.setPower(-0.6 * 1.1);
//        robot.right1.setPower(-0.6);
//        while (robot.left1.isBusy()) {}
    }
    void composeTelemetry(){


        telemetry.addAction(new Runnable() {
            @Override
            public void run() {
                robot.angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                robot.gravity = robot.imu.getGravity();
            }
        });
        telemetry.addLine()
                .addData("status", new Func<String>() {
                    @Override
                    public String value() {
                        return robot.imu.getSystemStatus().toShortString();
                    }
                })
                .addData("calib", new Func<String>() {
                    @Override
                    public String value() {
                        return robot.imu.getCalibrationStatus().toString();
                    }
                });
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

        telemetry.addLine()
                .addData("grvty", new Func<String>() {
                    @Override
                    public String value() {
                        return robot.gravity.toString();
                    }
                })
                .addData("mag", new Func<String>() {
                    @Override
                    public String value() {
                        return String.format(Locale.getDefault(), "%.3f",
                                Math.sqrt(robot.gravity.xAccel * robot.gravity.xAccel
                                        + robot.gravity.yAccel * robot.gravity.yAccel
                                        + robot.gravity.zAccel * robot.gravity.zAccel));
                    }
                });
    }
}